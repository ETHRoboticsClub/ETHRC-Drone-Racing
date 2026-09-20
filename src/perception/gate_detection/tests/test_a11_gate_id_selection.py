#!/usr/bin/env python3
"""Desk-simulation for A1.1 gate-ID tracking.

Replicates the _select_world_T_gate logic exactly (copy-paste of the body)
and runs it against a synthetic drone trajectory that passes through every
gate on the 6-gate course. No ROS, no torch, no PnP -- pure SE(3) math.

Pass criteria:
  - At each drone position along the track, the selected gate ID matches
    the actually-closest gate to the drone (which is what we'd want
    published if the model is locked on it).
  - On detection dropouts (no detection for N frames), the next detection
    still resolves to the right gate using the stale prior, as long as the
    drone hasn't moved more than ~half-a-gate-spacing in the meantime.
  - Bootstrap correctly returns gate 0.
"""
from __future__ import annotations
import math
import numpy as np


# --- Parkour map (mirrors config/parkour_map.yaml) -----------------------
GATES_NED = [
    np.array([ -23.298,   -0.400,   -0.032]),
    np.array([ -46.894,   -2.500,    5.068]),
    np.array([ -74.594,    1.200,   13.668]),
    np.array([-111.494,   -5.100,   24.568]),
    np.array([-135.494,   -0.800,   25.356]),
    np.array([-159.194,   -4.400,   25.968]),
]


def _se3(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _invert_se3(T):
    Ri = T[:3, :3].T
    ti = -Ri @ T[:3, 3]
    return _se3(Ri, ti)


def world_T_gate(i):
    """Identity rotation per the all-gates-same-orientation convention."""
    return _se3(np.eye(3), GATES_NED[i])


# --- Replicated selector (mirrors detector_node._select_world_T_gate) ----
class Selector:
    SUSPECT_M = 10.0  # match the patched detector_node value (was 14)
    PRIOR_MAX_AGE_NS = int(0.5e9)  # 0.5 s

    def __init__(self, start_gate_id: int = 0):
        self._parkour_gates = [world_T_gate(i) for i in range(len(GATES_NED))]
        self._last_world_T_body = None
        self._last_world_T_body_stamp_ns = None
        self._warned_bootstrap = False
        self._last_far_warn_ns = -10 * self.PRIOR_MAX_AGE_NS
        self.start_gate_id = start_gate_id
        self.warns = []
        self.now_ns = 0  # simulated wall clock

    def tick_ns(self, dt_ns):
        self.now_ns += int(dt_ns)

    def set_prior(self, world_T_body):
        """Publish-path: store the prior with the current simulated timestamp."""
        self._last_world_T_body = world_T_body
        self._last_world_T_body_stamp_ns = self.now_ns

    def _warn(self, msg):
        self.warns.append(msg)

    def select(self, cam_T_gate, cam_T_body):
        if not self._parkour_gates:
            self._warn("no map; falling back to identity")
            return np.eye(4)

        # Stale-prior check
        prior_is_fresh = (
            self._last_world_T_body is not None
            and self._last_world_T_body_stamp_ns is not None
            and (self.now_ns - self._last_world_T_body_stamp_ns) <= self.PRIOR_MAX_AGE_NS
        )
        if (
            self._last_world_T_body is not None
            and not prior_is_fresh
        ):
            age = (self.now_ns - self._last_world_T_body_stamp_ns) / 1e9
            self._warn(f"stale prior: age={age:.2f}s; rebootstrap to gate {self.start_gate_id}")
            self._last_world_T_body = None
            self._last_world_T_body_stamp_ns = None

        if self._last_world_T_body is None:
            if not self._warned_bootstrap:
                self._warn(f"bootstrap: returning gate {self.start_gate_id}")
                self._warned_bootstrap = True
            return self._parkour_gates[self.start_gate_id]

        body_T_cam = _invert_se3(cam_T_body)
        pred = self._last_world_T_body @ body_T_cam @ cam_T_gate
        pred_pos = pred[:3, 3]
        map_positions = np.array([g[:3, 3] for g in self._parkour_gates])
        dists = np.linalg.norm(map_positions - pred_pos[None, :], axis=1)
        gate_id = int(np.argmin(dists))
        closest = float(dists[gate_id])
        if closest > self.SUSPECT_M:
            # Rate-limited (1 Hz)
            if self.now_ns - self._last_far_warn_ns >= int(1e9):
                self._warn(f"far match: {closest:.1f} m to gate {gate_id}")
                self._last_far_warn_ns = self.now_ns
        return self._parkour_gates[gate_id]


# --- Synthetic flight -----------------------------------------------------
def fly_through(drone_xyz_world: np.ndarray, looking_at_gate_id: int):
    """Return (cam_T_body, cam_T_gate) for a drone at `drone_xyz_world`
    pointing at gate `looking_at_gate_id`.

    We use the FPV body frame convention (body +x = camera optical z),
    matching the detector_node stub. For simplicity the drone is
    'pointed' straight along world -X (south, toward the gates) — the
    rotation is exact for this contrived test.
    """
    # body_T_world: drone frame orientation in world. With identity rotation,
    # body frame = world frame. We choose body=world for the test (drone
    # is upright and oriented like world). The body translation in world
    # is drone_xyz_world.
    world_T_body = _se3(np.eye(3), drone_xyz_world.astype(np.float64))

    # FPV cam_T_body: optical z = body +x, optical x = body -y, optical y = body -z.
    # body_T_cam = inverse: body+x = cam+z, body+y = cam-x, body+z = cam-y.
    # R_body_cam: cols are [body-image of cam_x, cam_y, cam_z]
    #   cam_x ->  body -y
    #   cam_y ->  body -z
    #   cam_z ->  body +x
    R_body_cam = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    body_T_cam = _se3(R_body_cam, np.zeros(3))
    cam_T_body = _invert_se3(body_T_cam)

    world_T_cam = world_T_body @ body_T_cam
    cam_T_world = _invert_se3(world_T_cam)

    world_T_gate_i = world_T_gate(looking_at_gate_id)
    cam_T_gate = cam_T_world @ world_T_gate_i
    return cam_T_body, cam_T_gate, world_T_body


# --- Closest gate to drone (oracle) ---------------------------------------
def oracle_closest_gate(drone_xyz):
    dists = [np.linalg.norm(g - drone_xyz) for g in GATES_NED]
    return int(np.argmin(dists))


# --- Test 1: bootstrap returns start_gate_id -----------------------------
def test_bootstrap():
    sel = Selector(start_gate_id=0)
    drone = GATES_NED[0] + np.array([5.0, 0.0, 0.0])
    cam_T_body, cam_T_gate, _ = fly_through(drone, looking_at_gate_id=0)
    out = sel.select(cam_T_gate, cam_T_body)
    chosen = int(np.argmin([np.linalg.norm(out[:3, 3] - g) for g in GATES_NED]))
    ok = (chosen == 0) and any("bootstrap" in w for w in sel.warns)
    print(f"[test_bootstrap] picked gate {chosen} (expected 0)  "
          f"warns={sel.warns}  -> {'PASS' if ok else 'FAIL'}")
    return ok


# --- Test 2: nominal trajectory along course -----------------------------
def test_nominal_traverse():
    """Drone follows the great circle gate-0 -> gate-5 (linear interp).
    At each step, model 'sees' the closest gate. Each select() is followed
    by a set_prior(truth) (the publish-path side effect), with simulated
    33 ms wall-clock advance so the prior stays fresh.
    """
    sel = Selector(start_gate_id=0)
    N_PER_SEG = 12
    failures = 0
    total = 0
    initial = GATES_NED[0] + np.array([5.0, 0.0, 0.0])
    cam_T_body, cam_T_gate, world_T_body_truth = fly_through(initial, looking_at_gate_id=0)
    sel.select(cam_T_gate, cam_T_body)
    sel.set_prior(world_T_body_truth)
    sel.tick_ns(33e6)
    total += 1

    for seg in range(len(GATES_NED) - 1):
        a, b = GATES_NED[seg], GATES_NED[seg + 1]
        for k in range(1, N_PER_SEG + 1):
            t = k / N_PER_SEG
            drone = (1 - t) * a + t * b
            looking_at = seg if t < 0.5 else seg + 1
            cam_T_body, cam_T_gate, world_T_body_truth = fly_through(drone, looking_at_gate_id=looking_at)
            selected = sel.select(cam_T_gate, cam_T_body)
            sel.set_prior(world_T_body_truth)
            sel.tick_ns(33e6)
            chosen = int(np.argmin([np.linalg.norm(selected[:3, 3] - g) for g in GATES_NED]))
            total += 1
            if chosen != looking_at:
                failures += 1
                print(f"  seg {seg} t={t:.2f}: drone={drone.round(1)} "
                      f"looking_at gate {looking_at}, chose {chosen}")
    ok = failures == 0
    print(f"[test_nominal_traverse] {total} samples, {failures} mismatches  "
          f"warns={len(sel.warns)} -> {'PASS' if ok else 'FAIL'}")
    return ok


# --- Test 3: short dropout (5 frames @ 30 Hz, well within PRIOR_MAX_AGE) ---
def test_short_dropout():
    """5-frame dropout (~167 ms) is shorter than PRIOR_MAX_AGE_NS=500ms,
    so the prior is still 'fresh' and the proximity matcher uses it."""
    sel = Selector(start_gate_id=0)
    drone_approach = GATES_NED[2] + np.array([1.5, 0.0, 0.0])
    cam_T_body, cam_T_gate, world_T_body_truth = fly_through(
        drone_approach, looking_at_gate_id=2
    )
    sel.select(cam_T_gate, cam_T_body)
    sel.set_prior(world_T_body_truth)
    sel.tick_ns(5 * 33e6)  # 5-frame dropout = 167 ms

    drone_new = drone_approach + np.array([-1.7, 0.0, 0.3])
    cam_T_body, cam_T_gate, _ = fly_through(drone_new, looking_at_gate_id=2)
    selected = sel.select(cam_T_gate, cam_T_body)
    chosen = int(np.argmin([np.linalg.norm(selected[:3, 3] - g) for g in GATES_NED]))
    no_stale_warn = not any("stale" in w for w in sel.warns)
    ok = (chosen == 2) and no_stale_warn
    print(f"[test_short_dropout] picked gate {chosen} (expected 2), "
          f"no stale-prior warn -> {'PASS' if ok else 'FAIL'}")
    return ok


# --- Test 4: long dropout (>0.5s) forces stale-prior WARN + rebootstrap --
def test_long_dropout_rebootstraps():
    """Reviewer finding #2: a long dropout should be detected and trigger
    rebootstrap, not silently mis-assign via the telescoping fixed point."""
    sel = Selector(start_gate_id=0)
    # Prime at gate 2
    drone = GATES_NED[2] + np.array([1.5, 0.0, 0.0])
    cam_T_body, cam_T_gate, world_T_body_truth = fly_through(drone, looking_at_gate_id=2)
    sel.select(cam_T_gate, cam_T_body)
    sel.set_prior(world_T_body_truth)
    # Simulate a 2-second dropout (4x PRIOR_MAX_AGE_NS)
    sel.tick_ns(2 * 1e9)

    # New detection while the drone is now near gate 3
    drone_g3 = GATES_NED[3] + np.array([1.5, 0.0, 0.0])
    cam_T_body, cam_T_gate, _ = fly_through(drone_g3, looking_at_gate_id=3)
    selected = sel.select(cam_T_gate, cam_T_body)
    chosen = int(np.argmin([np.linalg.norm(selected[:3, 3] - g) for g in GATES_NED]))
    stale_warn_fired = any("stale" in w for w in sel.warns)
    bootstrap_warn_fired = any("bootstrap" in w for w in sel.warns)
    # After rebootstrap we land on start_gate_id=0 (wrong from drone's
    # actual position, but the WARN tells the operator what happened —
    # better than silently telescoping to a wrong but residual-0 match).
    ok = stale_warn_fired and bootstrap_warn_fired and chosen == 0
    print(f"[test_long_dropout_rebootstraps] picked gate {chosen} "
          f"(rebootstrap to 0 expected); "
          f"stale_warn={stale_warn_fired}  bootstrap_warn={bootstrap_warn_fired}  "
          f"-> {'PASS' if ok else 'FAIL'}")
    return ok


# --- Test 5: wrong bootstrap is a stable fixed point (documented hazard) ---
def test_wrong_bootstrap_is_fixed_point():
    """Reviewer finding #1: confirm that bootstrapping to the wrong gate
    is unrecoverable in-band. Drone is actually at gate 3 but operator set
    start_gate_id=0. Selector should lock onto gate 0 and stay there with
    residual 0 (no SUSPECT_M warn). This is the failure mode we cannot
    self-correct from — the only mitigation is "set the parameter right"."""
    sel = Selector(start_gate_id=0)  # WRONG: drone actually at gate 3
    drone_g3 = GATES_NED[3] + np.array([1.5, 0.0, 0.0])
    chosen_seq = []
    residuals = []
    for step in range(8):
        cam_T_body, cam_T_gate, world_T_body_truth = fly_through(drone_g3, looking_at_gate_id=3)
        selected = sel.select(cam_T_gate, cam_T_body)
        # Drone publishes its wrong pose (would happen in real ROS): the
        # next prior is the wrong world_T_body, not the truth.
        wrong_world_T_body = selected @ _invert_se3(cam_T_gate) @ cam_T_body
        sel.set_prior(wrong_world_T_body)
        sel.tick_ns(33e6)
        chosen_seq.append(int(np.argmin([np.linalg.norm(selected[:3, 3] - g) for g in GATES_NED])))
        # Distance from this iteration's selected gate to where we'd have
        # actually wanted to land:
        residuals.append(float(np.linalg.norm(selected[:3, 3] - GATES_NED[3])))
    locked_to_zero = all(c == 0 for c in chosen_seq)
    no_far_warns = not any("far match" in w for w in sel.warns)
    ok = locked_to_zero and no_far_warns
    print(f"[test_wrong_bootstrap_is_fixed_point] chosen sequence: {chosen_seq}  "
          f"(expected all 0 = stuck at wrong bootstrap)  "
          f"far_warn_fired={not no_far_warns}  "
          f"-> {'PASS (failure mode confirmed)' if ok else 'FAIL'}")
    return ok


def main():
    print(f"=== A1.1 desk simulation on the 6-gate course ===")
    results = [
        ("bootstrap", test_bootstrap()),
        ("nominal_traverse", test_nominal_traverse()),
        ("short_dropout", test_short_dropout()),
        ("long_dropout_rebootstraps", test_long_dropout_rebootstraps()),
        ("wrong_bootstrap_is_fixed_point", test_wrong_bootstrap_is_fixed_point()),
    ]
    print()
    n_pass = sum(int(ok) for _, ok in results)
    print(f"=== {n_pass}/{len(results)} tests passed ===")
    for name, ok in results:
        print(f"  {'OK ' if ok else 'FAIL'}  {name}")
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
