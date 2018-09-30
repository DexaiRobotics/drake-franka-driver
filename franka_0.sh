#!/bin/bash

config=$HOME/catkin_ws/src/salad_bar_description/franka_0.yaml

./build/franka_plan_runner $config
