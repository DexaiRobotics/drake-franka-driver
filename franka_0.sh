#! /bin/bash
#set -euxo

if [[ "$#" == "0" ]]; then
  echo "Default behavior, using hostname: $HOSTNAME"
  echo "to choose yaml"
  hostname_first_letter="${HOSTNAME:0:1}"
  name_of_file="franka_${hostname_first_letter}.yaml"
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
    name_of_file=$1.yaml
fi

echo "File chosen: $name_of_file"

config=$HOME/catkin_ws/src/salad_bar_description/$name_of_file

echo "choose config file: $config"

echo "start franka_driver"
if ./build/franka_driver $config ; then
    echo "franka_driver started successfully"
else
    echo "franka_driver failed to start"
fi
