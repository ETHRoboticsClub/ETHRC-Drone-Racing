
#!/usr/bin/env python3
import serial
import struct
import time
import sys
import signal
import math

def send_msp_packet(port, cmd, payload):
    size = len(payload)
    cs = size ^ cmd
    for b in payload: cs ^= b
    msg = b'$M<' + struct.pack('<BB', size, cmd) + payload + struct.pack('<B', cs)
    try:
        port.write(msg)
        port.flush()
    except Exception as e:
        print(f"Error writing to port: {e}")

def get_rc_payload(roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4):
    return struct.pack('<8H', roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)

def run_persistent_control(usb_port_name='/dev/ttyACM0', uart_port_name='/dev/ttyTHS1'):
    print(f"--- Persistent MSP Controller ---")
    print(f"USB Port (Arming): {usb_port_name}")
    print(f"UART Port (Override): {uart_port_name}")

    usb_port = None
    uart_port = None

    try:
        usb_port = serial.Serial(usb_port_name, 115200, timeout=0.1)
        print(f"Successfully opened USB: {usb_port_name}")
    except Exception as e:
        print(f"FAILED to open USB port {usb_port_name}: {e}")

    try:
        uart_port = serial.Serial(uart_port_name, 1500000, timeout=0.1)
        print(f"Successfully opened UART: {uart_port_name}")
    except Exception as e:
        print(f"FAILED to open UART port {uart_port_name}: {e}")

    if not usb_port and not uart_port:
        print("No ports available. Exiting.")
        return

    print("\nStarting Control Loop. Press Ctrl+C to Disarm and Exit.")
    
    # 1. Arming sequence: Start with zero throttle and DISARMED for a moment
    print(">>> Initializing...")
    for _ in range(20):
        if usb_port:
            send_msp_packet(usb_port, 200, get_rc_payload(1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500))
        if uart_port:
            send_msp_packet(uart_port, 200, get_rc_payload(1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500))
        time.sleep(0.05)

    # 2. Arm and Enable Override
    print(">>> ARMING and Enabling MSP OVERRIDE...")
    try:
        while True:
            # Pulse Logic: Oscillate throttle between 1000 and 1200 at 0.5Hz
            t = time.time()
            pulse = int(100 * math.sin(2 * math.pi * 0.5 * t))
            throttle = 1100 + pulse

            # USB port handles the "Radio" states: Armed (AUX1=2000), Override Mode (AUX2=1000)
            # Keeping throttle at 1000 on USB to distinguish from UART pulses
            if usb_port:
                send_msp_packet(usb_port, 200, get_rc_payload(1500, 1500, 1000, 1500, 2000, 1000, 1500, 1500))
            
            # UART port handles the actual sticks: Dynamic throttle pulse
            if uart_port:
                send_msp_packet(uart_port, 200, get_rc_payload(1500, 1500, throttle, 1500, 2000, 1000, 1500, 1500))
            
            time.sleep(0.02) # 50Hz heartbeat
    except KeyboardInterrupt:
        print("\n!!! Interruption detected. Initiating Emergency Disarm...")
    finally:
        # 3. Cleanup: Mandatory DISARM for at least 0.5s
        for _ in range(10):
            if usb_port:
                send_msp_packet(usb_port, 200, get_rc_payload(1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500))
            if uart_port:
                send_msp_packet(uart_port, 200, get_rc_payload(1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500))
            time.sleep(0.05)
        
        if usb_port: usb_port.close()
        if uart_port: uart_port.close()
        print("Ports closed. Safe state secured.")

if __name__ == '__main__':
    usb = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    uart = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyTHS1'
    run_persistent_control(usb, uart)
