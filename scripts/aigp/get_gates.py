#!/usr/bin/env python3
import struct
import time
import sys
from pymavlink import mavutil

def main():
    connection_string = 'udpin:0.0.0.0:14550'
    print(f"Connecting to simulator on {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)
    
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Connected! Sending reset command to trigger track info...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        31000, # MAVLINK_CMD_SIM_RESET
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Listening for track info data...")
    
    track_chunks = {}
    expected_num_track_chunks = {}
    
    start_time = time.time()
    while time.time() - start_time < 15: # Timeout after 15 seconds
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
            
        msg_type = msg.get_type()
        
        if msg_type == "DATA_TRANSMISSION_HANDSHAKE":
            transfer_id = msg.width
            track_chunks[transfer_id] = {}
            expected_num_track_chunks[transfer_id] = msg.packets
            print(f"Received handshake for transfer ID {transfer_id}, expecting {msg.packets} packets.")
            
        elif msg_type == "ENCAPSULATED_DATA":
            raw_payload = bytes(msg.data)
            data_type = raw_payload[0]
            
            if int(data_type) == 2: # ENCAPSULATED_TRACK_INFO_MSG_ID
                # unpack header
                data_type, transfer_id = struct.unpack_from("<BH", raw_payload)
                if transfer_id not in expected_num_track_chunks:
                    continue
                
                payload_chunk = raw_payload[3:]
                track_chunks[transfer_id][msg.seqnr] = payload_chunk
                
                # Check if all chunks received
                if len(track_chunks[transfer_id]) == expected_num_track_chunks[transfer_id]:
                    print("All track data chunks received. Reassembling...")
                    full_payload = bytes()
                    for i in range(len(track_chunks[transfer_id])):
                        full_payload += track_chunks[transfer_id][i]
                    
                    # Parse track data
                    num_gates, = struct.unpack_from("<H", full_payload)
                    payload = full_payload[2:]
                    print(f"\nFound {num_gates} gates in the track:")
                    print("-" * 80)
                    print(f"{'Gate ID':<9} | {'Position (NED) [X, Y, Z]':<30} | {'Width':<6} | {'Height':<6}")
                    print("-" * 80)
                    for idx in range(num_gates):
                        gate_id, px, py, pz, qw, qx, qy, qz, w, h = struct.unpack_from("<Hfffffffff", payload)
                        payload = payload[38:]
                        print(f"{gate_id:<9} | [{px:8.3f}, {py:8.3f}, {pz:8.3f}] | {w:<6.2f} | {h:<6.2f}")
                    print("-" * 80)
                    sys.exit(0)

    print("Timeout reached without receiving track data. Please restart the simulator/race to trigger track info transmission.")

if __name__ == "__main__":
    main()
