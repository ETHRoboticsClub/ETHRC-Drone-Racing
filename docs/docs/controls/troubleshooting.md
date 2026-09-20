# General Control Troubleshooting

**Page Owner:** [Mattia De Martino](https://github.com/mattiademartino)

---

## Overview

This pages provides solutions for common errors.

### 1.  Port is bound
Disconnect from container and do this:
```sh
$ sudo lsof -i :50051
COMMAND   PID       USER   FD   TYPE DEVICE SIZE/OFF NODE NAME
code    62707 papaveneti   42u  IPv4 762084      0t0  TCP localhost:50051->localhost:39538 (CLOSE_WAIT)
$ kill 62707 # copy the PID 