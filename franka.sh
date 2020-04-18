#! /bin/bash
#set -euxo

config=franka.yaml

echo "choose config file: $config"

echo "start franka_driver"
if ./build/franka_driver $config ; then
    echo "franka_driver started successfully"
else
    echo "franka_driver failed to start"
fi
