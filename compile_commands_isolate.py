#!/usr/bin/python3
"""
Isolate the compile commands for the git diff files.

Remove any other commands from the "escape" version of the exported json.
Intended for cppcheck, which goes through all build commands discarding
the positional arguments (file paths) when invoked from command line.
"""

import json

import git

path_json = "build/compile_commands_escape.json"
# get list of changed files in this repo
repo = git.Repo(".")
paths_diff = [item.a_path for item in repo.index.diff(None)]


def is_in_diff(abs_path):
    """Check if the given path is in the git diff list."""
    for path in paths_diff:
        if path in abs_path:
            return True
    return False


if __name__ == "__main__":
    # load compile commands json
    with open(path_json, "r") as f:
        cmds = json.load(f)
    # remove irrelevant entries
    i = 0
    while i < len(cmds):
        if is_in_diff(cmds[i]["file"]):
            i += 1
        else:
            cmds.pop(i)
    with open(path_json, "w") as f:
        json.dump(cmds, f, indent=4)
