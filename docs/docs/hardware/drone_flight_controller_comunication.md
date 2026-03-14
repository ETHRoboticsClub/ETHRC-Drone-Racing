# Port Jeston to Flight Controller Connection

The port used from the Jetson to connect to the Flight Controller is the following:

```bash
/dev/ttyTHS1
```

with **baudrate** 57600.

## Check connection 
To check that the connection is working run the following command:

```bash
mavproxy.py --master=/dev/ttyTHS1 --baudrate=57600
```
