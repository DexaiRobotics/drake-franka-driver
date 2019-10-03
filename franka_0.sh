#! /bin/sh -
set -euxo pipefail

if [[ "$#" == "0" ]]; then
  echo "Default behavior, using hostname: $HOSTNAME"
  echo "to choose yaml"
  name_of_file="franka_$(echo $HOSTNAME | head -c 1)"
else 
    # idiomatic parameter and option handling in sh
    case "$1" in
        franka_g) echo "option $1"
            ;;
        franka_h) echo "option $1"
            ;;
        franka_i) echo "option $1"
            ;;
        franka_j) echo "option $1"
            ;;
        franka_k) echo "option $1"
            ;;
        franka_l) echo "option $1"
            ;;
        franka_*) echo "robot name $1 does not exist! Exiting..."
            exit 1
            ;;
        *) echo "As the first argument, please supply a robot name "
            echo "franka_g, ..., franka_l! You supplied : $1! Exiting ..."
            exit 1
    esac
    name_of_file=$1
fi

config=$HOME/catkin_ws/src/salad_bar_description/$name_of_file.yaml

echo "choose config file: $config"

# if another driver instance is still somehow running, kill it silently
pkill -f franka_plan_runner > /dev/null 2>&1

./build/franka_plan_runner $config
