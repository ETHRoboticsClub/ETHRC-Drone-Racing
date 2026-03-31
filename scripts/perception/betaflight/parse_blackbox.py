import sys

def parse(filename):
    with open(filename, "rb") as f:
        data = f.read()
    
    total = len(data)
    i_frames = 0
    p_frames = 0
    
    # Simple marker-based counting
    for i in range(len(data) - 1):
        if data[i] == 0x49: # 'I' frame
            i_frames += 1
        elif data[i] == 0x50: # 'P' frame
            p_frames += 1
            
    print(f"File: {filename} ({total} bytes)")
    print(f"I-frames: {i_frames}")
    print(f"P-frames: {p_frames}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 parse_blackbox.py <file.bin>")
    else:
        parse(sys.argv[1])
