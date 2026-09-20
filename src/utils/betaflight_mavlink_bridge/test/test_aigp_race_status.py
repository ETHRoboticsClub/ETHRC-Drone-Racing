import struct

from betaflight_mavlink_bridge.aigp_race_status import parse_race_status_payload


def test_parse_race_status_payload():
    payload = struct.pack(
        "<BQqqIq",
        1,
        1234,
        1000,
        -1,
        3,
        987654321,
    )

    status = parse_race_status_payload(payload)

    assert status.sim_boot_time_ms == 1234
    assert status.race_start_boot_time_ms == 1000
    assert status.race_finish_time_ns == -1
    assert status.active_gate_index == 3
    assert status.last_gate_race_time == 987654321
