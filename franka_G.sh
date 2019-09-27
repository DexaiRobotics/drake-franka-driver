#!/bin/bash

# TODO: @robert set version in yaml & query
# TODO: @robert source yaml from host machine via web rather than github
config=$HOME/catkin_ws/src/salad_bar_description/franka_G.yaml

# if another driver instance is still somehow running, kill it silently
pkill -f franka_plan_runner > /dev/null 2>&1

./build/franka_plan_runner $config
