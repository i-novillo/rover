# Installing external dependencies
## Pigpio
* Go to the pigpio folder `pigpio`
* Unzip the pigpio library contents with `unzip master.zip`
* Compile the library `make` and install  it `sudo make install`
* You can check the installation with `pigpiod -v`
* The _pigpio daemon_ needs to be started to run pigpio. You can start it by running `sudo pigpiod`, and you can stop the daemon with `sudo killall pigpiod`