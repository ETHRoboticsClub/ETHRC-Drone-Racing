#!/usr/bin/env python3
import serial
import struct
import time
import sys

def manage_override(port_name='/dev/ttyACM0', duration=5):
    """
    Safely handles the MSP Override sequence:
    1. Arms the drone (AUX1=2000)
    2. Enables MSP Override (AUX2=1000)
    3. Maintains pulse for 'duration' seconds
    4. Disables Override and Disarms for safety
    """
    print(f"Connecting to MSP RX port {port_name} at 115200 baud...")
    try:
        port = serial.Serial(port_name, 115200, timeout=0.1)
    except Exception as e:
        print(f"FAILED to open port {port_name}: {e}")
        print("Check if the port is correct (e.g. /dev/ttyACM0 or /dev/ttyUSB0)")
        return

    def send_channels(roll, pitch, throttle, yaw, aux_val):
        # AETR Mapping + AUX1..4
        # Setting all AUX channels to aux_val (ARM/DISARM) to match test_msp.py logic
        payload = struct.pack('<8H', roll, pitch, throttle, yaw, aux_val, aux_val, aux_val, aux_val)
        size = len(payload)
        cmd = 200 # MSP_SET_RAW_RC
        cs = size ^ cmd
        for b in payload: cs ^= b
        msg = b'$M<' + struct.pack('<BB', size, cmd) + payload + struct.pack('<B', cs)
        try:
            port.write(msg)
            port.flush()
        except Exception as e:
            print(f"Write error: {e}")

    try:
        print(">>> 1. ARMING (AUX1-4=2000)...")
        for _ in range(40): # ~2 seconds
            send_channels(1500, 1500, 1000, 1500, 2000)
            time.sleep(0.05)

        print(">>> 2. MAINTAINING ARM STATE...")
        for _ in range(40): # ~2 seconds
            send_channels(1500, 1500, 1000, 1500, 2000)
            time.sleep(0.05)

        print(f">>> 3. MODE ACTIVE: Running for {duration} seconds...")
        print("    (Motors should be spinning if props are off)")
        start_time = time.time()
        while time.time() - start_time < duration:
            # Send low throttle pulse while keeping ARMED (2000 on AUX channels)
            send_channels(1500, 1500, 1100, 1500, 2000)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n!!! Manual Interruption Detected !!!")
    finally:
        print(">>> 4. CLEANUP: Disarming...")
        # Send 2 seconds of safety-off commands (AUX=1000)
        for _ in range(40):
            send_channels(1500, 1500, 1000, 1500, 1000)
            time.sleep(0.05)
        port.close()
        print("SAFE STATE SECURED. Connection Closed.")

if __name__ == "__main__":
    # Check for CLI arguments
    port_arg = '/dev/ttyACM0'
    duration_arg = 5
    
    if len(sys.argv) > 1:
        port_arg = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            duration_arg = int(sys.argv[2])
        except:
            pass
            
    manage_override(port_arg, duration_arg)
