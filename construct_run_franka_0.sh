#! /bin/bash
set -euxo

cd build
make -j4 franka_plan_runner
cd ..

./franka_0.sh
