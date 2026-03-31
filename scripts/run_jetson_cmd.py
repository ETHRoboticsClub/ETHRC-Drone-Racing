import pty
import os
import sys
import select

def run_ssh(cmd, password):
    ssh_cmd = ["ssh", "-o", "StrictHostKeyChecking=no", "aero@192.168.55.1", cmd]
    
    pid, fd = pty.fork()
    
    if pid == 0:  # Child process
        os.execvp("ssh", ssh_cmd)
    else:  # Parent process
        output = b""
        password_sent = False
        
        while True:
            r, _, _ = select.select([fd], [], [], 5)
            if not r:
                break  # Timeout
            
            try:
                data = os.read(fd, 1024)
            except OSError:
                break
                
            if not data:
                break
                
            output += data
            print(data.decode('utf-8', errors='ignore'), end="")
            sys.stdout.flush()
            
            if b"password" in data.lower() and not password_sent:
                os.write(fd, (password + "\n").encode())
                # Don't set password_sent to True immediately to allow for sudo if needed,
                # but prevent infinite loop on fast streams if we keep seeing the prompt.
                password_sent = True
                
        os.waitpid(pid, 0)
        return output.decode('utf-8', errors='ignore')

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 run_ssh.py '<command>'")
        sys.exit(1)
        
    cmd = sys.argv[1]
    res = run_ssh(cmd, "strongpass")
    print(res)
