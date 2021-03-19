#!/bin/bash

set -eufo pipefail

# parse --dev option for developer mode
OPT_DEV=false
while (( $# )); do
  case "$1" in
    --dev)
      OPT_DEV=true
      shift
      ;;
    *)
      break
  esac
done

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

if [[ $OPT_DEV == true ]]; then
  echo "Instaling git pre-commit hooks assuing all dependencies are present..."
  git config --unset-all core.hooksPath
  git lfs install
  pip install -U pre-commit pylint
  pre-commit install
  echo "dev setup complete"
  exit 0
fi


if (( $build_tests > 0 )); then
    target=all
else
    target=franka_driver    # build only the main library
fi
echo "####### make will use $num_threads jobs to build target: $target #######"

echo "update libfranka and build if not done yet..."
pushd "$(dirname $(realpath $0))"


if (( $clean_build > 0 )); then
    if [ -d "build" ]; then
        rm -rf build
    fi
fi

if [ ! -f "build/libfranka.so" ]; then
    pushd externals/libfranka
    echo "build libfranka..."
    mkdir -p build && cd build
    if (( $build_debug > 0 )); then
        echo "Build libfranka in Debug mode!"
        cmake .. -DCMAKE_BUILD_TYPE=Debug \
                 -DCMAKE_C_COMPILER=gcc-7 \
                 -DCMAKE_CXX_COMPILER=g++-7
    else
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_C_COMPILER=gcc-7 \
                 -DCMAKE_CXX_COMPILER=g++-7
    fi
    cmake --build . -j $num_threads --target franka communication_test
    popd
fi


echo "clean_build = $clean_build"
if (( $clean_build > 0 )); then
    #if the CMakeCache.txt file exists, remove it.
    echo "Performing a clean build!"
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
    echo "Building in debug mode"
    cmake .. -DCMAKE_BUILD_TYPE=Debug   || exit 4   # Build for debugging
else
    echo "Building in release mode"
    cmake .. -DCMAKE_BUILD_TYPE=Release || exit 5
fi

make  -j $num_threads $target           || exit 6
if (( $exec_ctests > 0 )); then
    if (( $skip_slower )); then
        ctest -E $exclude_pat           || exit 7   # exclude tests matching the regex
    else
        ctest                           || exit 8   # run all unit tests
    fi
fi
popd
