# drake-franka-driver

[![CircleCI](https://circleci.com/gh/DexaiRobotics/drake-franka-driver.svg?style=shield&circle-token=a122ea0349f6e79f84f549e6155bbbfcf923d7d4)](https://circleci.com/gh/DexaiRobotics/drake-franka-driver)

Drake-compatible LCM driver for the Franka PANDA research robot. This driver uses the [Franka Control Interface](https://frankaemika.github.io/docs/) and is designed for use with the [Drake](https://drake.mit.edu/) planning and control toolbox (currently up to the 2020/05/30 build).

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
git clone --recursive https://github.com/DexaiRobotics/drake-franka-driver.git`
drake-franka-driver/setup.sh
```

See the beginning of `setup.sh` for more flags.

## Running the driver

Once built, the executable `drake-franka-driver` can be found in the `build/` directory. In order to run the driver, execute:
```bash
drake-franka-driver <robot_name> <robot_ip_address>
```
where the default robot_name is `franka_0` and the default IP is `192.168.200.0`. This enables multiple robots to be run on the same network.

## Communicating with the robot

### Sending commands to the robot from a drake program

The driver listens for commands on the `<robot_name>_cmd` LCM channel. By default this is `franka_0_cmd`.

### listening to the robot response

The `drake-franka-driver` reports the robot status on the `<robot_name>_status` lcm channel. By default this is set to `franka_0_status`. Status is reported in a `franka_status` struct.

## Starting driver on NUC

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