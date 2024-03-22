# drake-franka-driver

[![CircleCI](https://circleci.com/gh/DexaiRobotics/drake-franka-driver.svg?style=shield&circle-token=a122ea0349f6e79f84f549e6155bbbfcf923d7d4)](https://circleci.com/gh/DexaiRobotics/drake-franka-driver)

Drake-compatible LCM driver for the Franka PANDA research robot. This driver uses the [Franka Control Interface](https://frankaemika.github.io/docs/) and is designed for use with the [Drake](https://drake.mit.edu/) planning and control toolbox.

## Prerequisites

This C++ driver depends on `drake` and `drake-lcmtypes`. You may use our [`drake-torch` images](https://github.com/DexaiRobotics/drake-torch) or install `drake` with the following.

```bash
curl -SL https://drake-packages.csail.mit.edu/drake/nightly/drake-20200530-bionic.tar.gz | tar -xzC /opt
cd /opt/drake/share/drake/setup && yes | ./install_prereqs
```

See [drake-shambhala](https://github.com/RobotLocomotion/drake-shambhala) for more examples.

You also need the LCM library.
```bash
git clone https://github.com/lcm-proj/lcm
cd lcm && mkdir -p build && cd build && cmake ..
make install -j 12
```

`drake` has deprecated some lcmtypes needed here. The `robot_msgs` [repo](https://github.com/DexaiRobotics/robot_msgs) has a copy of them; only the headers in `include/lcmtypes` are needed. See `.circleci/config.yml` and `CMakeLists.txt` for details.

## Building the driver

```bash
git clone --recursive https://github.com/DexaiRobotics/drake-franka-driver.git
cd drake-franka-driver
./setup.sh
```

See the beginning of [`setup.sh`](setup.sh) for more flags.

## Running the driver

Once built, the executable `drake-franka-driver` can be found in the `build/` directory. In order to run the driver, execute:

```bash
./franka.sh
```

To run the driver in simulated mode without connecting to real Franka hardware, run the following:

```bash
export SIM_ROBOT=true
./franka.sh
```

## Communicating with the robot

### Sending commands to the robot from another program via LCM

The driver listens for plans encoded as a piecewise polynomial of LCM type [`robot_msgs::robot_spline_t`](https://github.com/DexaiRobotics/robot_msgs/blob/master/lcmtypes/robot_spline_t.lcm) on the `<FRANKA_ID>_PLAN` LCM channel. By default the name is `FRANKA_` plus the capitalized first letter of the hostname of the computer running the `drake-franka-driver`. The `FRANKA_ID` can be overridden by setting the `robot_name` field in the parameters file.

Currently active plans can be paused or canceled with a [`robot_msgs::pause_cmd`](https://github.com/DexaiRobotics/robot_msgs/blob/master/lcmtypes/pause_cmd.lcm) LCM message via the `<FRANKA_ID>_STOP` LCM channel.

If the driver is running in simulated mode, it is possible to simulate driver events like Franka control exceptions, or pressing the user stop button, on the `<FRANKA_ID>_SIM_EVENT_TRIGGER` LCM channel.

### Listening to the robot response

The driver publishes the following LCM channels with information about the current status:
- `<FRANKA_ID>_STATUS` of type [`drake::lcmt_iiwa_status`](https://github.com/RobotLocomotion/drake/blob/master/lcmtypes/lcmt_iiwa_status.lcm) with information about joint positions, velocities, etc.
- `<FRANKA_ID>_ROBOT_STATUS` of type [`robot_msgs::robot_status_t`](https://github.com/DexaiRobotics/robot_msgs/blob/master/lcmtypes/robot_status_t.lcm) which includes more comprehensive information that's included in the `franka::RobotState` struct including end effector forces
- `<FRANKA_ID>_DRIVER_STATUS` of type [`robot_msgs::driver_status_t`](https://github.com/DexaiRobotics/robot_msgs/blob/master/lcmtypes/driver_status_t.lcm) which contains high level information about the driver, if it is currently running a plan, if it's paused or user stopped, etc.

## Starting driver

In our implementation, the NUC runs this driver to talk to the Franka Controller. There are 3 ways to start the driver.

### Using `dexai-cli` from beast machine

On the beast machine, run

```bash
dexai-cli run -c ful -v <desired_deploy_docker_version>
```

Note: this will bring up the entire system on the desired version, including the Franka driver and AA driver on the NUC, and the TLE container and deploy container for di/ROS services on the beast machine.

### Using `./run_drivers.sh` script on NUC

Open an ssh session into the NUC and do the following:
```bash
cd ~/fullstack
./run_drivers.sh --tag <desired_deploy_docker_version>
# example: ./run_drivers.sh --tag 7.1.56
```

### Building from source

Sometimes, building from source is required in order to test a new feature in the driver. Open an ssh session into the NUC and do the following:
```bash
docker kill franka-driver-deploy # if there is currently a driver running
cd ~/dev_fullstack
git checkout <your_feature_branch>
./build_fullstack_detached.sh -b
```
Once the build completes, you can enter the running docker container and manually start the driver:
```bash
cd /src/drake-franka-driver
./franka.sh
```