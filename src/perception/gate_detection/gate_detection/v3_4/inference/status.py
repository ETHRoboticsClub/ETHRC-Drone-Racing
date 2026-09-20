"""Per-frame tracker status dataclass.

Extracted from DroneRacing_GateDetection's ``inference/tracker.py`` so the
hybrid + seg-CC trackers don't drag the full :class:`GateTracker` class
(and its ``losses.keypoint.coherent_quad_candidates`` dependency) into
the ROS package. Field set is identical to the upstream dataclass.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class GateTrackerStatus:
    """Per-frame diagnostic snapshot returned alongside the picked quad."""

    n_candidates: int = 0
    picked_rect_score: Optional[float] = None
    picked_proximity: Optional[float] = None
    picked_diagonal: Optional[float] = None
    track_was_held: bool = False
    fail_count: int = 0
    handoff_triggered: bool = False
    reason: str = "ok"
