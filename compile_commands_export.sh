#!/bin/bash

# Copyright © 2020 Dexai Robotics. All rights reserved.

# this script configures the cmake project and generates
# two versions of compile_commands.json
# one is the original version, which has triple escapes by ROS
# the other has triple escaped ROS_PACKAGE_NAME macros replaced
# e.g. -DROS_PACKAGE_NAME=\\\"robot_interface\\\"
# becomes -DROS_PACKAGE_NAME=\"robot_interface\"

src_dir="$(dirname $(realpath $0))"
pushd "${src_dir}/build" > /dev/null

if [ ! -f "./compile_commands.json" ]; then
  cmake \
    -D CMAKE_BUILD_TYPE:STRING=Debug \
    -D CMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc-10 \
    -D CMAKE_CXX_COMPILER:FILEPATH=/usr/bin/g++-10 \
    -H ".." \
    -B "." \
    -G "Unix Makefiles"
fi

echo "Fixing escapes in ROS macros for cppcheck..."
# using . as delimiters, every first \ is an escape character
sed 's.\\\\\\\"..g' "compile_commands.json" > "compile_commands_escape.json"
echo "Compile commands exported."
cd ..
./compile_commands_isolate.py
echo "Compile commands isolated."
popd > /dev/null
