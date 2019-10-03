#! /bin/sh -

set -euxo pipefail

if [[ "$#" == "0" ]]; then
  echo "Please supply as argument a robot name such as franka_G."
  exit 1
fi

# idiomatic parameter and option handling in sh
case "$1" in
        franka_G) echo "option $1"
            ;;
        franka_H) echo "option $1"
            ;;
        franka_I) echo "option $1"
            ;;
        franka_J) echo "option $1"
            ;;
        franka_K) echo "option $1"
            ;;
        franka_L) echo "option $1"
            ;;
        franka_*) echo "robot name $1 does not exist! Exiting"
            exit 1
            ;;
        *) echo "As the first argument, please supply a robot name "
            echo "franka_G, ..., franka_L! You supplied : $1"
            exit 1
esac

config=$HOME/catkin_ws/src/salad_bar_description/$1.yaml

# if another driver instance is still somehow running, kill it silently
pkill -f franka_plan_runner > /dev/null 2>&1

./build/franka_plan_runner $config
