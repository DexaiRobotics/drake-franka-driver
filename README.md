# drake-franka-driver
Drake-compatible LCM driver for the Franka PANDA research robot. This driver uses the [Franka Control Interface](https://frankaemika.github.io/docs/) and is designed for use with the [Drake](https://drake.mit.edu/) planning and control toolbox. 
# c++ driver
## cmake build process for Ubuntu and macOS
Note: the c++ drake-franka-driver depends on `drake` and `drake-lcmtypes` so it must be compiled as a `drake` cmake-external project - see [drake-shambhala](https://github.com/RobotLocomotion/drake-shambhala) for more information. 
### pre-requisites
1. Install `lcm` system-wide. We recommend `brew install lcm`
2. Install [Drake](https://drake.mit.edu/) to `/opt/drake/` as is standard 
*NOTE* ignore steps 1 and 2 if running inside docker
In order to build the driver, use the following steps:

3. `git clone https://github.com/DexaiRobotics/drake-franka-driver.git`
4. `cd drake-franka-driver && git submodule update --init`
5. `cd externals/libfranka && git submodule update --init`
6. `cd .. && ../setup.sh` (if using docker, run inside docker image)

When running `setup.sh` if you get an error that the cmake for Franka cannot be found, try removing `libfranka/build` and rerunning setup

## running the driver
Once built, the executable `drake-franka-driver` can be found in the `build/` directory. In order to run the driver, execute:
`./drake-franka-driver <robot_name> <robot_ip_address>` 
where the default robot_name is `franka_0` and the default ip is `192.168.200.0` which enables multiple robots to be run on the same network. 
## communicating with the robot
### sending commands to the robot from a drake program
The `drake-franka-driver` listens for commands on the `<robot_name>_cmd` lcm channel. By default this is set to `franka_0_cmd`.
### listening to the robot response
The `drake-franka-driver` reports the robot status on the `<robot_name>_status` lcm channel. By default this is set to `franka_0_status`. Status is reported in a `franka_status` struct.


