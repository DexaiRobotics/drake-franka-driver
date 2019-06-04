#!/bin/bash

# Positional Parameters for specifying different behavior via the command line.
# see http://linuxcommand.org/lc3_wss0120.php
# Example CircleCI usage (from .circleci/config.yml): ./setup.sh 4 1 1
num_threads=${1:-8} # Num threads for make -j.  For CircleCI, try using 4.
build_tests=${2:-1} # Build the tests?  Yes if build_tests > 0 or exec_ctests > 0.
exec_ctests=${3:-1} # Run ctest if all else succeeds?   Defaults to Yes (1 > 0).
build_debug=${4:-0} # Make build type Debug (with symbols) ctest if > 0.  Defaults to No (0).
clean_build=${5:-1} # Remove the build directory and start over if > 0.  Defaults to Yes (1).
skip_slower=${6:-0} # Skip the slow tests, such as test_foi (or all matching 'test_[A-Z].+').  Defaults to No (0).
exclude_pat=${7:-test_multiple_plans} # Regex for excluding some (slow) tests, such as test_foi.

if (( $build_tests > 0 )); then
    target=all
else
    target=drake-franka-driver    # build only the main library
fi
echo "####### make will use $num_threads jobs to build target: $target #######"

source scripts/setup_env.sh
# git submodule update --init

cd externals
git submodule update --init --recursive
cd libfranka
if [ ! -d "build" ]; then
  mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
  cmake --build . -j $num_threads --target franka
  cd ..
fi
cd ../..

if (( $clean_build > 0 )); then
    #if the CMakeCache.txt file exists, remove it.
    if [ -f CMakeCache.txt ]; then
      rm -f CMakeCache.txt
    fi
    #if the build directory exists, remove it.
    if [ -d build ]; then
      rm -rf build
    fi
fi

# Now build the target from scratch:
mkdir -p build; cd build
if (( $build_debug > 0 )); then
    cmake .. -DCMAKE_BUILD_TYPE=Debug || exit 4   # Build for debugging
else
    cmake ..                          || exit 5
fi

make  -j $num_threads $target         || exit 6
if (( $exec_ctests > 0 )); then
    if (( $skip_slower )); then
        ctest -E $exclude_pat         || exit 7   # exclude tests matching the regex
    else
        ctest                         || exit 8   # run all unit tests
    fi
fi
