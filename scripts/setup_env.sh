#!/bin/bash
# set build environment for StuffGetter

export DRAKE_BUILD_DIR="/opt/drake"
export FRANKA_BUILD_DIR="$PWD/externals/libfranka/build"
export DRACULA_SOURCE_PATH="$PWD/../dracula"
export DRACULA_BUILD_PATH="$DRACULA_SOURCE_PATH/build"
export DRACULA_INCLUDE_DIR="$DRACULA_SOURCE_PATH/dracula/include"
export DRACULA_LIBRARIES="$DRACULA_BUILD_PATH/dracula/libdracula.so"
