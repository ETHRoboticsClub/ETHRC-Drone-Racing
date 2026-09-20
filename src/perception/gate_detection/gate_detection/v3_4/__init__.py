"""v3.4 hybrid gate-detection pipeline (DeepLabV3+ seg + kp + next-gate heads).

Ported verbatim from DroneRacing_GateDetection commit 14d21f3
(report-15.5 working tree). Only the file subset reachable from
:class:`gate_detection.v3_4.inference.predictor.GatePredictor.predict_full_from_bgr`
and :class:`gate_detection.v3_4.inference.hybrid_tracker.HybridSegNextGateTracker`
is included — see ``handover.md`` for the dep-tree audit.
"""
