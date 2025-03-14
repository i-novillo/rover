## IMPORTANT
* For the pigpio module to work, the daemon needs to be running in an IPv4 interface not IPv6 ("You can check where it is listening, by starting pigpiod and then running "netstat -vatn" in a terminal."). For that, when starting the daemon do `sudo pigpiod -n 127.0.0.1`

* If bluetooth is not working, it's possible the BT interface needs to be set to discoverable. For that, do `bluetoothctl` and then `discoverable on`