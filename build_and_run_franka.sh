#! /bin/bash
set -eux

cd build
make -j4 franka_driver
cd ..

./franka.sh
