#! /bin/bash
#set -euxo

if [[ "$#" == "0" ]]; then
  echo "Default behavior, using hostname: $HOSTNAME"
  echo "to choose yaml"
  name_of_file="franka_$(echo $HOSTNAME | head -c 1)"
  echo "File chosen: $name_of_file"
else 
    # idiomatic parameter and option handling in sh
    case "$1" in
        franka_i) echo "option $1"
            ;;
        franka_j) echo "option $1"
            ;;
        franka_k) echo "option $1"
            ;;
        franka_l) echo "option $1"
            ;;
        franka_m) echo "option $1"
            ;;
        franka_n) echo "option $1"
            ;;
        franka_*) echo "robot name $1 does not exist! Exiting..."
            exit 1
            ;;
        *) echo "As the first argument, please supply a robot name "
            echo "franka_i, ..., franka_n! You supplied something else: $1! Exiting ..."
            exit 1
    esac
    name_of_file=$1
fi

config=$HOME/catkin_ws/src/salad_bar_description/$name_of_file.yaml

echo "choose config file: $config"

# if another driver instance is still somehow running, kill it silently
pkill -f franka_plan_runner > /dev/null 2>&1
echo "start franka_plan_runner"
if ./build/franka_plan_runner $config ; then
    echo "franka_plan_runner started successfully"
else
    echo "franka_plan_runner failed to start"
fi
