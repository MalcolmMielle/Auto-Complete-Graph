#!/bin/bash
# source /opt/ros/melodic/setup.bash

# RUN THIS SCRIPT FROM `your-workspace/src/`
vcs import < Auto-Complete-Graph/rosinstall
sudo apt update
rosdep install --from-paths . --ignore-src -r -y

# Stop g2o from complaining about old cmake version
g2o_cmake="libraries/g2o/CMakeLists.txt"
if [ -f $g2o_cmake ]
then
    sed  -ie '/cmake_minimum_required/s/3.14/3.10/' $g2o_cmake
else
    echo -e "\e[31mWhere is $g2o_cmake?\e[0m"
fi

pushd libraries/BetterGraph && mkdir -p build && cd build && cmake .. && sudo make install
popd
pushd libraries/VoDiGrEx && mkdir -p build && cd build && cmake .. && sudo make install
popd
pushd libraries/g2o && mkdir -p build && cd build && cmake .. && sudo make install
popd

catkin build
