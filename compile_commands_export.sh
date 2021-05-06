#!/bin/bash

# BSD 3-Clause License
#
# Copyright (c) 2021, Dexai Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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

echo "Removing warning option '-Wshadow=local' unknown to clang..."
sed 's/ -Wshadow=local//' build/compile_commands-diff_only.json -i

popd > /dev/null
