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

## Docker Cross-Compilation of ROS
* The idea is to use the multi-platform build concept explained in ROS2 Humble docs: `https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html` 
* Docker needs to be installed `https://docs.docker.com/desktop/setup/install/linux/`
* For the initial setup, go to the raspberry_pi folder to build the docker image using `docker buildx build --platform linux/arm64 -t ros2_aarch64:latest --load .`
* From then on, cross-compile can be performed by launching the script `raspi_deployment.sh`
* Cross-compile requires the Raspberry to be on, as binaries are automatically copied to it via SCP. IP, directories and Username need to be updated in the deployment script.
* If you get errors related to `exec /bin/bash: exec format error` ensure QEMU is properly setup in the host machine with: `sudo apt-get install qemu-user-static` and then `sudo update-binfmts --enable`