# Installing external dependencies

## Compiler and Make
* Install `GCC` along with other tools  `sudo apt install build-essential`
* Install `make` using `sudo apt install make`

## spdlog
* Install spdlog library `sudo apt-get install libspdlog-dev`

## Pigpio
* Go to the pigpio folder `cd pigpio`
* Unzip the pigpio library contents with `unzip master.zip`
* Compile the library `make` and install it `sudo make install`
* You can check the installation with `pigpiod -v`
* The _pigpio daemon_ needs to be started to run pigpio. You can start it by running `sudo pigpiod`, and you can stop
  the daemon with `sudo killall pigpiod`
