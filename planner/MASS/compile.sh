#!/bin/bash

while getopts "c:" flag; do
    case "$flag" in
    c) CPLEX_DIR=$OPTARG ;;
    *) echo "Invalid option. ${USAGE}" ;;
    esac
done

if [ -z "${CPLEX_DIR}" ]; then
    CPLEX_DIR_ARGS=""
else
    CPLEX_DIR_ARGS="-DCPLEX_BASE_DIR=${CPLEX_DIR}"
fi

mkdir build

# build exec for cpp

cd build
cmake ${CPLEX_DIR_ARGS} ../
cpuCores=$(cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}')
make -j $cpuCores
