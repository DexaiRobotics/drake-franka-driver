#!/usr/bin/python3
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

"""
Isolate the compile commands for the git diff files.

Remove any other commands from the "escape" version of the exported json.
Intended for cppcheck, which goes through all build commands discarding
the positional arguments (file paths) when invoked from command line.
"""

import json
import os

import git

build_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build")
# get list of changed files in this repo
repo = git.Repo(".")
paths_diff = [item.a_path for item in repo.index.diff(None)]


def is_in_diff(abs_path):
    """Check if the given path is in the git diff list."""
    for path in paths_diff:
        if path in abs_path:
            return True
    return False


def filter_json(path_input):
    """Isolate compile commands for only the git diff ones given a json."""
    with open(path_input, "r") as f:  # load compile commands json
        cmds = json.load(f)
    i = 0  # remove irrelevant entries
    while i < len(cmds):
        if is_in_diff(cmds[i]["file"]):
            i += 1
        else:
            cmds.pop(i)
    with open(path_input.replace(".json", "-diff_only.json"), "w") as f:
        json.dump(cmds, f, indent=4)


def gen_path(filename):
    """Generate absolute path to compile commands json given filename."""
    return os.path.join(build_dir, filename)


if __name__ == "__main__":
    filter_json(gen_path("compile_commands.json"))
    filter_json(gen_path("compile_commands_escape.json"))
