# Installing external dependencies

## Compiler and Make
* Install `GCC` along with other tools  `sudo apt install build-essential`
* Install `make` using `sudo apt install make`
* For cross-compilation, install the aarch64-linux-gnu toolchain packages `sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu`

## spdlog
* Clone the `spdlog` repository in the `external_dependencies` folder `git clone https://github.com/gabime/spdlog.git`
* The paths in the `CMake` configurations should be adjusted to look for the correct `spdlog` path

## Pigpio
* Go to the pigpio folder `cd pigpio`
* Unzip the pigpio library contents with `unzip master.zip`
* Adjust the `CROSS_PREFIX` flag in `pigpio-master/Makefile` for cross-compilation if necessary
* Compile the library `make` and install it `sudo make install`
* You can check the installation with `pigpiod -v`
* The _pigpio daemon_ needs to be started to run pigpio. You can start it by running `sudo pigpiod`, and you can stop
  the daemon with `sudo killall pigpiod`
