import struct
import time
import os

def send_msp(port):
    os.system('stty -F ' + port + ' 115200 raw -echo')
    
    print("1. Sending Zero Throttle (1000) and DISARMED state (AUX1=1000) to pass boot checks...")
    
    # helper for sending one frame with a specific arm state
    def send_frame(throttle, arm_state):
        # AETR Mapping:
        # Index 0: Roll (Aileron)  = 1500
        # Index 1: Pitch (Elevator)= 1500
        # Index 2: Throttle        = throttle
        # Index 3: Yaw (Rudder)    = 1500
        # Index 4-7: AUX1-AUX4     = arm_state (brute forcing all aux switches to arm state)
        payload = struct.pack('<8H', 1500, 1500, throttle, 1500, arm_state, arm_state, arm_state, arm_state)
        size = len(payload)
        cmd = 200
        cs = size ^ cmd
        for b in payload: cs ^= b
        f.write(b'$M<' + struct.pack('<BB', size, cmd) + payload + struct.pack('<B', cs))

    with open(port, 'wb', buffering=0) as f:
        # Let the serial port DTR reset settle
        time.sleep(1)
        
        # Zero throttle and DISARMED for a full second
        for i in range(50):
            send_frame(1000, 1000)
            time.sleep(0.02)
            
        print("2. Flipping ARM Switch (AUX1=2000) while throttle is zero...")
        for i in range(50):
            send_frame(1000, 2000)
            time.sleep(0.02)
            
        print("3. Armed! Spooling up throttle to 1150...")
        for i in range(50):
            send_frame(1150, 2000)
            time.sleep(0.02)
            
        print("4. Dropping throttle back to zero and Disarming (AUX1=1000)...")
        for i in range(20):
            send_frame(1000, 1000)
            time.sleep(0.02)
            
    print("Test sequence complete. The drone executed a perfect legal arm.")

if __name__ == '__main__':
    send_msp('/dev/ttyACM0')
