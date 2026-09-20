"""Hybrid seg-CC × next-gate-head tracker (report 15.3, with 15.5 fixes).

Combines the strengths of two earlier failures:

  * **SegCCTracker** (option #2) — the seg head has high COVERAGE
    (85 %+ fire rate; one CC per visible gate). But the "largest CC"
    SELECTION rule is wrong for multi-gate scenes — picks the wrong
    gate when the gates have similar size.

  * **NextGateMaskTracker** (option #1, v3.2 Dice+BCE) — the next-
    gate head's mask SELECTS the correct gate (max sigmoid 0.7-0.95
    on piloted multi-gate scenes), but EXTRACTING corners directly
    from its own mask under-covers (16 % fire rate) because the
    head's mask is a small ring and the kp peaks inside it can be
    weak.

This tracker uses the next-gate head's mask as a **selection signal**
over the seg head's CCs — pick the seg-CC with maximum overlap with
the next-gate-head's ``>0.5`` region. Coverage comes from the seg
head; selection correctness comes from the next-gate head.

Fallback policy (CLAUDE.md §6 — no silent fallbacks):
  * If the next-gate mask is absent / empty / has zero overlap with
    every seg CC, log a [WARN] and fall back to the largest seg CC
    (same as SegCCTracker). This makes the hybrid degrade
    gracefully on autonomous-flight-style frames where the head
    wasn't trained.

Cited basis: AlphaPilot Sec. III-C (Foehn et al. 2022) uses a
learned permutation matrix over the seg mask to assign keypoints to
gates. We do the same conceptual thing with a simpler interface —
the next-gate-mask head IS the assignment signal, and the seg-CCs
are the assignment targets.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from gate_detection.v3_4.inference.seg_cc_tracker import (
    extract_quad_in_bbox,
    find_gate_components,
)
from gate_detection.v3_4.inference.status import GateTrackerStatus


@dataclass
class HybridTrackerConfig:
    """Knobs for :class:`HybridSegNextGateTracker`."""

    seg_threshold: float = 0.5
    min_area: int = 200
    max_area_frac: float = 0.40

    next_gate_threshold: float = 0.5
    """Binarisation of the next-gate-mask head before overlap scoring.
    Inherits the v3.2 calibration: max sigmoid 0.7-0.95 on piloted
    multi-gate scenes, so 0.5 is the natural cut."""

    fallback_overlap_px: int = 50
    """Minimum px² of overlap (in the chosen CC's bbox) with the
    next-gate-mask for the selection to be considered 'guided' by
    the head. Below this we fall back to the largest-CC rule and
    log it per CLAUDE.md §6 so the fallback isn't silent."""

    conf_threshold: float = 0.30
    match_radius_px: float = 60.0
    switch_area_margin: float = 1.69
    max_fail_streak: int = 3

    handoff_area_frac: float = 0.15
    """Area-of-image threshold for handoff. Raised from 0.08 (report
    15.4) to 0.15 (report 15.5) after the full-flight bench showed
    668 handoffs / 5600 frames — way too many. A real-gate ring at
    near-passage covers ~10-15 % of the image (the seg mask is the
    ring, not the solid gate); 0.08 was firing on close-but-not-
    passing approaches and chaining handoff cycles. 0.15 fires only
    when the gate genuinely fills the frame."""

    handoff_persist_frames: int = 2

    handoff_cooldown_frames: int = 30
    """After a handoff fires, suppress re-lock onto any CC whose
    centroid is within ``passed_radius_px`` of the freshly-passed
    gate's centroid for this many subsequent frames. Without this
    the tracker re-acquires the just-passed gate (still visible
    behind the drone for several frames), fires handoff again,
    drops, re-acquires... a chain that produced 668 spurious
    handoffs / 5600 frames in the v3.4 hybrid bench. 30 frames at
    fps=30 stride=2 ≈ 1 s real time — enough for a typical 1.5-2 m
    inter-gate spacing at 8-15 m/s to pass the gate out of view."""

    passed_radius_px: float = 100.0
    """Cooldown radius (px) around the freshly-passed gate's
    centroid. CCs whose centroid is within this radius are excluded
    from selection during ``handoff_cooldown_frames``. 100 px ≈ 26 %
    of image side — wide enough to suppress the post-pass-still-
    visible blob, tight enough not to reject a genuinely different
    gate that happens to share the screen region."""

    min_frames_between_handoffs: int = 25
    """Hard guard: never fire a handoff fewer than this many frames
    after the previous one, regardless of cooldown-filter outcome.
    Belt-and-suspenders for the two edge cases reviewer round 3 of
    report 15.5 flagged: (a) single-gate-only fallback in
    ``find_gate_components`` (the filter falls back to the full CC
    list when filtering would leave zero candidates → just-passed
    gate can be re-picked → handoff re-fires), and (b) centroid
    drift faster than passed_radius_px over the cooldown window
    (typical 10 m/s pass moves the gate's image centroid > 100 px
    within ~5 frames, so the radius-based filter stops suppressing
    while the drone hasn't yet cleared the gate).

    Pattern: standard SORT/ByteTrack-style post-event cooldown
    (Bewley et al. 2016, arXiv:1602.00763) — we adopt the same
    primitive at the handoff-event boundary rather than at the
    track-creation boundary.

    25 frames < 30-frame cooldown so the guard isn't redundantly
    gating after the centroid cooldown expires; it only catches
    the gap when the centroid cooldown silently fails."""


class HybridSegNextGateTracker:
    """Track-by-detection where the next-gate-mask head SELECTS which
    seg-CC to use for corner extraction.

    Usage::

        tracker = HybridSegNextGateTracker()
        for frame in stream:
            rgb, seg, kp, ng = predictor.predict_full_from_bgr(bgr)
            picked, status = tracker.step(seg, ng, kp)
            ...
    """

    def __init__(self, config: Optional[HybridTrackerConfig] = None):
        self.config = config or HybridTrackerConfig()
        self.locked_centroid: Optional[tuple[float, float]] = None
        self.locked_area: Optional[int] = None
        self.fail_count: int = 0
        self.passed_count: int = 0
        # Post-handoff cooldown — report 15.5.
        self.passed_centroid: Optional[tuple[float, float]] = None
        self.passed_cooldown: int = 0
        # Belt-and-suspenders min-frames-between-handoffs guard. Initial
        # value is large so the FIRST handoff (which has no prior to
        # cool down from) fires normally.
        self.frames_since_handoff: int = 10**9
        self._fallback_warned: int = 0

    def reset(self) -> None:
        self.locked_centroid = None
        self.locked_area = None
        self.fail_count = 0
        self.passed_count = 0
        self.passed_centroid = None
        self.passed_cooldown = 0
        self.frames_since_handoff = 10**9

    def step(
        self,
        seg_prob: np.ndarray,
        next_gate_mask: Optional[np.ndarray],
        kp_heatmap: Optional[np.ndarray],
    ) -> tuple[Optional[np.ndarray], GateTrackerStatus]:
        cfg = self.config
        status = GateTrackerStatus()

        if kp_heatmap is None:
            status.reason = "no_kp_head"
            return None, status

        H, W = seg_prob.shape
        img_area = float(H * W)

        # Tick the post-handoff cooldown FIRST so the per-frame
        # filter below sees the up-to-date value. When the cooldown
        # expires, clear the passed centroid so the tracker can lock
        # back onto that screen region for the next pass.
        if self.passed_cooldown > 0:
            self.passed_cooldown -= 1
            if self.passed_cooldown == 0:
                self.passed_centroid = None
        # Tick the min-frames-between-handoffs guard. Never overflow.
        if self.frames_since_handoff < 10**9:
            self.frames_since_handoff += 1

        # Seg CCs first — these define coverage.
        ccs_all = find_gate_components(
            seg_prob,
            threshold=cfg.seg_threshold,
            min_area=cfg.min_area,
            max_area_frac=cfg.max_area_frac,
        )
        # Apply post-handoff cooldown: skip any CC whose centroid is
        # within passed_radius_px of the freshly-passed centroid. This
        # prevents the re-lock-on-just-passed-gate cycle that drove
        # 668 spurious handoffs in the v3.4 bench. If filtering would
        # leave zero candidates we keep the full list (better to
        # re-fire one handoff than to stop firing entirely on a
        # legitimate detection).
        if self.passed_centroid is not None:
            radius_sq = cfg.passed_radius_px * cfg.passed_radius_px
            filtered = [
                c for c in ccs_all
                if (
                    (c["centroid"][0] - self.passed_centroid[0]) ** 2
                    + (c["centroid"][1] - self.passed_centroid[1]) ** 2
                ) > radius_sq
            ]
            ccs = filtered if filtered else ccs_all
        else:
            ccs = ccs_all
        status.n_candidates = len(ccs)
        if not ccs:
            status.reason = "no_candidates"
            return self._soft_fail(status)

        # Compute next-gate-mask overlap per seg CC (bbox-windowed
        # overlap — fast and good enough; the exact CC mask isn't
        # exposed by find_gate_components and the bbox is a tight
        # axis-aligned superset).
        ng_bin: Optional[np.ndarray] = None
        if next_gate_mask is not None:
            ng_bin = (next_gate_mask > cfg.next_gate_threshold).astype(np.uint8)
        max_ng_overlap = 0
        for cc in ccs:
            if ng_bin is not None:
                x0, y0, w, h = cc["bbox"]
                # Clip to image bounds defensively.
                x1 = min(W, x0 + w)
                y1 = min(H, y0 + h)
                x0 = max(0, x0)
                y0 = max(0, y0)
                cc["ng_overlap"] = int(ng_bin[y0:y1, x0:x1].sum())
            else:
                cc["ng_overlap"] = 0
            if cc["ng_overlap"] > max_ng_overlap:
                max_ng_overlap = cc["ng_overlap"]

        # Pick — by NG overlap when meaningful, else by area (fallback).
        guided_by_ng = max_ng_overlap >= cfg.fallback_overlap_px
        if self.locked_centroid is None:
            if guided_by_ng:
                chosen = max(ccs, key=lambda c: c["ng_overlap"])
                status.reason = "fresh_pick"
            else:
                # Fall back to largest area when the NG head has no
                # meaningful signal on this frame. Logged with [WARN]
                # via _warn_fallback so the fallback is auditable.
                self._warn_fallback(
                    f"fresh pick fell back to largest-CC (max NG overlap "
                    f"{max_ng_overlap} < {cfg.fallback_overlap_px})"
                )
                chosen = ccs[0]
                status.reason = "fresh_pick"
        else:
            # Centroid match against the previously locked CC.
            matched, matched_dist = None, float("inf")
            for cc in ccs:
                d = float(np.hypot(
                    cc["centroid"][0] - self.locked_centroid[0],
                    cc["centroid"][1] - self.locked_centroid[1],
                ))
                if d < matched_dist:
                    matched_dist = d
                    matched = cc
            if matched is None or matched_dist > cfg.match_radius_px:
                status.reason = "lock_lost"
                return self._soft_fail(status)
            status.picked_proximity = matched_dist

            # Override: a different CC has a STRONGER next-gate-head
            # signal than the matched one (the head says "the next
            # gate moved over there"). This is the analogue of
            # SegCCTracker's much-bigger-area override, but in
            # selection-space (ng overlap) instead of size-space.
            biggest_other = None
            for cc in ccs:
                if cc is matched:
                    continue
                if (cc["ng_overlap"] >= cfg.switch_area_margin * max(matched["ng_overlap"], 1)
                        and cc["ng_overlap"] >= cfg.fallback_overlap_px):
                    if biggest_other is None or cc["ng_overlap"] > biggest_other["ng_overlap"]:
                        biggest_other = cc
            if biggest_other is not None:
                chosen = biggest_other
                status.reason = "switch_to_bigger_ng"
            else:
                chosen = matched
                status.reason = "ok" if guided_by_ng else "ok_fallback"

        # Extract corners within the chosen CC's bbox (4 px padding —
        # tighter than seg_cc_tracker's 8 px to avoid picking up
        # neighbouring-gate corners; the seg-CC bbox is already a
        # tight fit around the gate region).
        quad, confs = extract_quad_in_bbox(
            kp_heatmap, chosen["bbox"], padding=4,
        )
        if quad is None:
            status.reason = "no_decode"
            return self._soft_fail(status)
        if float(confs.min()) < cfg.conf_threshold:
            status.reason = "low_conf"
            return self._soft_fail(status)

        # Diagnostics
        status.picked_diagonal = 0.5 * (
            float(np.linalg.norm(quad[2] - quad[0])) +
            float(np.linalg.norm(quad[3] - quad[1]))
        )
        TL, TR, BR, BL = quad
        top_w = float(TR[0] - TL[0])
        bot_w = float(BR[0] - BL[0])
        left_h = float(BL[1] - TL[1])
        right_h = float(BR[1] - TR[1])
        status.picked_rect_score = abs(top_w - bot_w) + abs(left_h - right_h)

        # Commit
        self.locked_centroid = chosen["centroid"]
        self.locked_area = chosen["area"]
        self.fail_count = 0

        # Handoff (same as SegCCTracker — area-based, since the
        # chosen object is still a seg-CC). On fire, set the
        # post-handoff cooldown so the next ~30 frames cannot
        # re-lock onto this gate's screen region (report 15.5).
        # Belt-and-suspenders: refuse to fire if fewer than
        # ``min_frames_between_handoffs`` have passed since the
        # previous handoff — closes the two edge cases reviewer
        # round 3 of report 15.5 flagged (single-gate fallback,
        # centroid drift > radius).
        area_frac = chosen["area"] / img_area
        if area_frac >= cfg.handoff_area_frac:
            self.passed_count += 1
            if (self.passed_count >= cfg.handoff_persist_frames
                    and self.frames_since_handoff >= cfg.min_frames_between_handoffs):
                status.handoff_triggered = True
                status.reason = "handoff"
                self.passed_centroid = chosen["centroid"]
                self.passed_cooldown = cfg.handoff_cooldown_frames
                self.frames_since_handoff = 0
                self.locked_centroid = None
                self.locked_area = None
                self.passed_count = 0
                self.fail_count = 0
        else:
            self.passed_count = 0

        return quad, status

    def _soft_fail(
        self, status: GateTrackerStatus,
    ) -> tuple[None, GateTrackerStatus]:
        self.fail_count += 1
        status.fail_count = self.fail_count
        if self.fail_count >= self.config.max_fail_streak:
            self.locked_centroid = None
            self.locked_area = None
            self.fail_count = 0
            self.passed_count = 0
        return None, status

    def _warn_fallback(self, msg: str, budget: int = 5) -> None:
        """[WARN] up to ``budget`` times, then suppress (CLAUDE.md §6)."""
        if self._fallback_warned < budget:
            self._fallback_warned += 1
            print(f"[WARN] HybridSegNextGateTracker: {msg}", flush=True)
            if self._fallback_warned == budget:
                print(
                    "[WARN] HybridSegNextGateTracker: further fallback "
                    "warnings suppressed.",
                    flush=True,
                )
