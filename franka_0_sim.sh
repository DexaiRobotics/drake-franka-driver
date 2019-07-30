#!/bin/bash

config=$HOME/catkin_ws/src/salad_bar_description/franka_0_sim.yaml

# if another driver instance is still somehow running, kill it silently
pkill -f franka_plan_runner > /dev/null 2>&1

./build/franka_plan_runner $config
