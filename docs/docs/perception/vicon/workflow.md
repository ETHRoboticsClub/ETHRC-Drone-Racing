# Vicon — workflow

**Owner:** [Jules Schmutz](https://github.com/juliomathis)

## Arena setup

At least five MX-F40 cameras cover a cylinder of roughly 6 m diameter and 5 m height; seven are recommended for the full 8 × 8 × 5 m arena. The cameras are old and hard to repair — power the hub off before touching any cable.

1. Mount the cameras on tripods in a ring, pointing at the centre; add strain relief to every cable.
2. Route the cables to the Vicon hub, connect them, then connect the hub to the Windows 11 PC by gigabit Ethernet.
3. Power the hub on (silver button, front). Start **Vicon Tracker 3** (Tracker 4 does not work with these cameras).
4. Camera LEDs turn green within a minute. In Tracker, select all cameras and open *Camera Preview* to see the infrared views. If this fails, power off and ask a lead.

The manufacturer's guide: [Vicon Tracker user guide (Prophysics)](https://www.prophysics.ch/wp-content/uploads/2017/06/Vicon-Tracker-User-Guide.pdf).

## Calibration (in Tracker 3)

1. **Mask** — *Calibrate* tab → *Auto Mask* with the volume empty. Removes static reflections.
2. **Wand wave** — *Calibrate Cameras* → select the wand → *Start*. Move the wand slowly through the whole volume, including depth, until Tracker reports completion. Redo if errors are high or a camera is missing.
3. **Origin** — place the L-frame on the floor where the origin should be, *Set Volume Origin* → select the L-frame → *Start* → *Set Origin*. The grid in the 3D view should now match the floor.

## Create the drone object

In Tracker: create an object from the drone's markers and name it after the Jetson hostname (`zephyrus`, `boreas`). The segment name defaults to the subject name, giving the topic `/vicon/zephyrus/zephyrus` — which is what `racing_bringup.launch.py` expects by default.

## Run

Through the bringup (default `use_vicon:=true`, Vicon PC at `192.168.0.78`):

```bash
ros2 launch drone_control racing_bringup.launch.py msp_bench_mode:=true
```

Standalone, or with a different Vicon PC:

```bash
ros2 launch vicon_receiver client.launch.py hostname:=192.168.0.78 \
  world_frame:=map vicon_frame:=vicon map_xyz:='[0.0, 0.0, 0.0]' map_rpy:='[0.0, 0.0, 0.0]'
```

The bringup hard-codes the hostname; change it in `racing_bringup.launch.py` if the Vicon PC moves.

## Verify

```bash
ros2 topic list | grep vicon
ros2 topic hz /vicon/$(hostname)/$(hostname)      # Vicon frame rate
ros2 topic echo --once /vicon/$(hostname)/$(hostname)
ros2 run tf2_ros tf2_echo map vicon
```

Walk the drone along +x of the L-frame and confirm `position.x` increases.

## Troubleshooting

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| No `/vicon/...` topics | Not connected, or object not visible | Check `hostname`, that the Jetson is on the Aero network, and that the object is tracked in Tracker |
| Topic exists but stops updating | Occlusion or object lost | Check marker visibility; the receiver does not signal loss |
| Pose jumps between two locations | Two objects share a marker layout | Recreate the object with a distinct marker pattern |
| Wrong topic name in the bringup | Hostname ≠ Tracker object name | Pass `vicon_subject:=... vicon_segment:=...` |
