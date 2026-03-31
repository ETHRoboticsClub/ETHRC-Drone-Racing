import pexpect
import sys

def execute_command(command, host, user="aero", password="strongpass"):
    ssh_cmd = f"ssh -o StrictHostKeyChecking=no {user}@{host} \"{command}\""
    child = pexpect.spawn(ssh_cmd, encoding='utf-8')
    
    try:
        i = child.expect(['password:', pexpect.EOF, pexpect.TIMEOUT], timeout=15)
        if i == 0:
            child.sendline(password)
            child.expect(pexpect.EOF)
            return child.before
        elif i == 1:
            return child.before
        else:
            return "Timeout"
    except Exception as e:
        return str(e)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 ssh_exec.py \"command\" [host]")
        sys.exit(1)
    cmd = sys.argv[1]
    host = sys.argv[2] if len(sys.argv) > 2 else "192.168.55.1"
    print(execute_command(cmd, host))
