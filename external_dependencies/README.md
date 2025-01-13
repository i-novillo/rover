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

## ROS2
* Follow the documentation in ROS2 installation page (for ubuntu 22.04): `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`
* Follow the environment setup section before running ROS: `https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html`
* Follow the colcon installation tutorial: `https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html`