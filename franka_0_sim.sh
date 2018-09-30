#!/bin/bash

config=$HOME/catkin_ws/src/salad_bar_description/franka_0_sim.yaml

./build/franka_plan_runner $config
