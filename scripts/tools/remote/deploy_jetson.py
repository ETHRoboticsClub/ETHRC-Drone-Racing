import sys
import base64
import os
from scripts.tools.remote.ssh_exec import execute_command

def deploy(local_file, remote_path, host="192.168.55.1"):
    with open(local_file, "rb") as f:
        content = f.read()
    
    b64_content = base64.b64encode(content).decode('utf-8')
    # Use chunked cat to avoid shell buffer issues
    chunk_size = 1000
    execute_command(f"rm -f {remote_path} && touch {remote_path}", host)
    
    for i in range(0, len(b64_content), chunk_size):
        chunk = b64_content[i:i+chunk_size]
        execute_command(f"echo -n '{chunk}' | base64 -d >> {remote_path}", host)
    
    print(f"Deployed {local_file} to {host}:{remote_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 deploy_jetson.py <local_file> <remote_path> [host]")
    else:
        deploy(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "192.168.55.1")
