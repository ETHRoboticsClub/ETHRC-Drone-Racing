import sys
import struct

def decode_active(filename):
    with open(filename, "rb") as f:
        data = f.read()
    
    idx = 0
    count = 0
    while idx < len(data) - 37:
        if data[idx] == 0x49: # 'I' marker
            # Intra frame usually has fixed structure in 4.5.x
            # Accel: [14:20], Gyro: [20:26]
            try:
                ax, ay, az = struct.unpack("<hhh", data[idx+14:idx+20])
                gx, gy, gz = struct.unpack("<hhh", data[idx+20:idx+26])
                print(f"[{count}] ACC: {ax:6}, {ay:6}, {az:6} | GYR: {gx:6}, {gy:6}, {gz:6}")
                count += 1
                idx += 37 # Skip frame
            except:
                idx += 1
        else:
            idx += 1

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 see_data.py <file.bin>")
    else:
        decode_active(sys.argv[1])
