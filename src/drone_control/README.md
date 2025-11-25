# Building the package

We need to downgrade the `setuptools` version we currently use
```sh
sudo python3 -m pip uninstall -y setuptools
sudo python3 -m pip install setuptools==79.0.1
```


# Fix port is bound:
Disconnect from container and do this:
```sh
$ sudo lsof -i :50051
COMMAND   PID       USER   FD   TYPE DEVICE SIZE/OFF NODE NAME
code    62707 papaveneti   42u  IPv4 762084      0t0  TCP localhost:50051->localhost:39538 (CLOSE_WAIT)
$ kill 62707 # copy the PID 
```
