#! /bin/bash

pushd /src/drake-franka-driver

if [[ ${SIM_ROBOT:-false} == 'false' ]]; then
    config=franka.yaml
else
    config=franka_sim.yaml
fi

# if another driver instance is still somehow running, kill it silently
pkill -f franka_driver > /dev/null 2>&1

if ./build/franka_driver $config ; then
    echo "franka_driver started successfully"
else
    echo "franka_driver failed to start"
fi

popd
