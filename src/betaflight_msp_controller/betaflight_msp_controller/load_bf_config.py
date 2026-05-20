#!/usr/bin/env python3
import sys
import serial
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description="Auto-load Betaflight configuration.")
    parser.add_argument('config_file', help="Path to the Betaflight .txt config file")
    parser.add_argument('--port', default='/dev/ttyTCU0', help="Serial port to connect to")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    try:
        with open(args.config_file, 'r') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"Error reading config file: {e}")
        sys.exit(1)

    print(f"Opening {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        sys.exit(1)

    # Wait for the port to initialize
    time.sleep(1)

    print("Entering CLI mode...")
    # Send multiple 'enter' and '#' to wake up CLI
    for _ in range(3):
        ser.write(b'\r')
        time.sleep(0.1)
        ser.write(b'#\r')
        time.sleep(0.1)
    
    # Flush existing output and check for prompt
    time.sleep(0.5)
    resp = ser.read_all().decode('utf-8', errors='ignore')
    if '#' not in resp:
        print("Warning: Did not see CLI prompt (#), attempting to proceed anyway...")

    print(f"Writing {len(lines)} lines from {args.config_file}...")
    for i, line in enumerate(lines):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
            
        # Send line
        ser.write((line + '\r').encode('utf-8'))
        
        # Small delay for processing
        time.sleep(0.02)
        
        # Every 10 lines, print progress
        if i % 10 == 0:
            print(f" Sent {i}/{len(lines)} lines...")
    
    print("Ensuring batch end and save...")
    ser.write(b'batch end\r')
    time.sleep(0.1)
    ser.write(b'save\r')
    time.sleep(2)
    
    ser.close()
    print("Configuration sequence sent successfully.")

if __name__ == '__main__':
    main()
