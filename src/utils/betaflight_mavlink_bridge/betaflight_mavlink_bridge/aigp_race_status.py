from __future__ import annotations

from dataclasses import dataclass
import struct


ENCAPSULATED_RACE_STATUS_MSG_ID = 1
ENCAPSULATED_TRACK_INFO_MSG_ID = 2
RACE_STATUS_STRUCT = struct.Struct("<BQqqIq")


@dataclass(frozen=True)
class AigpRaceStatus:
    data_type: int
    sim_boot_time_ms: int
    race_start_boot_time_ms: int
    race_finish_time_ns: int
    active_gate_index: int
    last_gate_race_time: int


def parse_race_status_payload(payload: bytes | bytearray | list[int]) -> AigpRaceStatus:
    raw = bytes(payload)
    if len(raw) < RACE_STATUS_STRUCT.size:
        raise ValueError(
            f"AIGP race-status payload must be at least {RACE_STATUS_STRUCT.size} bytes, got {len(raw)}"
        )

    values = RACE_STATUS_STRUCT.unpack_from(raw)
    status = AigpRaceStatus(*values)
    if status.data_type != ENCAPSULATED_RACE_STATUS_MSG_ID:
        raise ValueError(f"Unexpected AIGP encapsulated data type: {status.data_type}")
    return status
