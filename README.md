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

## Builidng the driver

```bash
git clone --recursive https://github.com/DexaiRobotics/drake-franka-driver.git`
drake-franka-driver/setup.sh
```

See the beginning of `setuo.sh` for more flags.

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

## Starting driver on NUC remotely
In our implementation, the NUC runs this driver to talk to the Franka Controller.
There are 3 ways to start the driver: either using `di start core:franka` command on the beast computer, using `franka` command directly on the beast computer, or manually starting the driver on the NUC. See also here: https://github.com/DexaiRobotics/wiki/wiki/Running-Candy-Demo#starting-drake-franka-driver-manually-to-debug
### franka command
There is a command called `franka` which is run on the Beast Computer (this is what `di start core:franka` callsunder the hood): https://github.com/DexaiRobotics/deploy/blob/master/franka
On the NUC, there is a crontab which runs on startup: https://github.com/DexaiRobotics/deploy/blob/master/robot_cron/start_docker_and_driver_servers.sh
This will:
1) make sure a docker is running and all code is built
2) start up the LCM servers for both Franka and AA https://github.com/DexaiRobotics/fullstack/blob/master/src/run_driver_servers.sh

The LCM server https://github.com/DexaiRobotics/drake-franka-driver/blob/master/scripts/franka_server.py will listen to LCM commands from the franka command on the beast computer, and start up the actual driver when requested.
