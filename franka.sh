#! /bin/bash

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

pushd /src/drake-franka-driver

if [[ ${SIM_ROBOT:-false} == 'false' ]]; then
    config=franka.yaml
else
    config=franka_sim.yaml
fi

# if another driver instance is still somehow running, kill it silently
pkill -f franka_driver > /dev/null 2>&1

# do not use $HOSTNAME as it is a variable that may have been set in a
# different environment or overridden
hostname_first_letter="$(echo $(hostname) | head -c 1)"
robot_name="FRANKA_${hostname_first_letter^}"
status_channel="${robot_name}_STATUS"
echo "Starting Franka driver with hostname: $(hostname), Franka name: ${robot_name}"

# check if there is already a driver running. at the start of this script
# we try to kill any existing instances, but if there are multiple containers
# running on the same machine, or have an instance running on another
# networked machine, we don't want to duplicate
if lcm-echo -n 1 -v 0 --timeout 0.1 ${status_channel}; then
    echo "A Franka driver instance is already running and publishing to ${status_channel}!"
    echo "Check other containers and networked machines to find out the source."
    exit 1
fi

./build/franka_driver $config $@

popd
