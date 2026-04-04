#!/usr/bin/env python3
import struct
import time
import sys
import serial

def send_msp(port_name, baudrate=1500000):
    print(f"Connecting to {port_name} at {baudrate} baud...")
    
    try:
        port = serial.Serial(port_name, baudrate, write_timeout=0)
    except Exception as e:
        print(f"Failed to open port {port_name}: {e}")
        return

    def send_frame(thrust, arm_state, aux2_state):
        # AETR Mapping + AUX1..4
        # Roll, Pitch, Thrust, Yaw, AUX1, AUX2, AUX3, AUX4
        payload = struct.pack('<8H', 1500, 1500, thrust, 1500, arm_state, aux2_state, 1500, 1500)
        size = len(payload)
        cmd = 200 # MSP_SET_RAW_RC
        cs = size ^ cmd
        for b in payload: cs ^= b
        msg = b'$M<' + struct.pack('<BB', size, cmd) + payload + struct.pack('<B', cs)
        try:
            port.write(msg)
            port.flush()
        except serial.SerialTimeoutException:
            pass

    try:
        print("1. Passing boot checks: Zero Throttle (1000), Disarmed (AUX1=1000), No Override (AUX2=1500)")
        for i in range(20):
            send_frame(1000, 1000, 1500)
            time.sleep(0.05)
            
        print("2. Arming the drone (AUX1=2000) while throttle is zero...")
        for i in range(20):
            send_frame(1000, 2000, 1500)
            time.sleep(0.05)
            
        print("3. Switching to MSP OVERRIDE MODE (AUX2=1000) while Armed")
        for i in range(20):
            send_frame(1000, 2000, 1000)
            time.sleep(0.05)
            
        print("4. Controlling Motors: Spooling up throttle to 1200...")
        for i in range(50):
            send_frame(1200, 2000, 1000)
            time.sleep(0.05)
            
        print("5. Dropping throttle to zero, Disarming (AUX1=1000), and Disabling MSP Override (AUX2=1500)")
        for i in range(20):
            send_frame(1000, 1000, 1500)
            time.sleep(0.05)
            
        print("Test sequence complete.")
    finally:
        port.close()

if __name__ == '__main__':
    port_arg = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyTHS1'
    baud_arg = int(sys.argv[2]) if len(sys.argv) > 2 else 1500000
    send_msp(port_arg, baud_arg)
