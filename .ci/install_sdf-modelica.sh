#!/bin/sh
# exit immediately if a command exits with a non-zero status
set -e

# lsb_release and wget are required to install the correct repositories, so we install it beforehand
apt-get update
apt-get install -y lsb-release wget gnupg

# get osrf repo
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Update packages
apt-get update

# noninteractive tzdata ( https://stackoverflow.com/questions/44331836/apt-get-install-tzdata-noninteractive ); tzdata = timezone data
export DEBIAN_FRONTEND=noninteractive

# CI specific packages
apt-get install -y clang git gcc g++

# SDF
apt-get install -y libsdformat6-dev

# script
cd $TRAVIS_BUILD_DIR
mkdir build && cd build
cmake -DBUILD_TESTING:BOOL=ON -G"${TRAVIS_CMAKE_GENERATOR}" ..
cmake --build . --config $TRAVIS_BUILD_TYPE
ctest --output-on-failure --build-config ${TRAVIS_BUILD_TYPE}
cmake --build . --config ${TRAVIS_BUILD_TYPE} --target install
