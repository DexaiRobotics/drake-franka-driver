#!/bin/bash

config=$HOME/catkin_ws/src/salad_bar_description/franka_sim.yaml

# if another driver instance is still somehow running, kill it silently
pkill -f franka_driver > /dev/null 2>&1

./build/franka_driver $config