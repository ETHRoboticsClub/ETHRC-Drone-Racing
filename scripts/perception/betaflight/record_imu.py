import sys
import serial
import time

def record(port="/dev/ttyTHS1", baud=1500000, duration=10, output="blackbox.bin"):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Recording from {port} for {duration}s...")
        with open(output, "wb") as f:
            start = time.time()
            total_bytes = 0
            while time.time() - start < duration:
                data = ser.read(1024)
                if data:
                    f.write(data)
                    total_bytes += len(data)
        print(f"Done! Saved {total_bytes} bytes to {output}")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    record()
