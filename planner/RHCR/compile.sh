#!/bin/bash

mkdir build

# build exec for cpp

cd build
cmake ../
cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`
make -j $cpuCores


# build exec for python

# cd build
# cmake ../ -DPYTHON=true
# make -j
# cp ./*.so ../  #make sue the library is in the working directory