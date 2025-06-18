# Compile everything in the project
target="$1"
current_path=$(pwd)
cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`

compile_rpclib() {
    echo "Compiling rpclib..."
    cd $current_path/client/externalDependencies/rpclib
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j $cpuCores
    sudo make install
}

compile_client() {
    echo "Compiling client..."
    cd $current_path/client
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j $cpuCores
}

compile_server() {
    echo "Compiling server..."
    cd $current_path/server
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j $cpuCores
}

compile_pbs() {
    echo "Compiling PBS..."
    cd $current_path/planner/PBS
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j $cpuCores
    sudo find $current_path/planner/PBS/build/pbs -type d -exec chmod o+x {} +
}

compile_rhcr() {
    echo "Compiling RHCR..."
    cd $current_path/planner/RHCR
    rm -rf build
    bash compile.sh
}

if [ "$target" == "rpclib" ]; then
    compile_rpclib
fi

if [ "$target" == "client" ]; then
    compile_client
fi

if [ "$target" == "server" ]; then
    compile_server
fi

if [ "$target" == "pbs" ]; then
    compile_pbs
fi

if [ "$target" == "rhcr" ]; then
    compile_rhcr
fi

if [ "$target" == "all" ]; then
    compile_rpclib
    compile_client
    compile_server
    compile_pbs
    compile_rhcr
fi

if [ "$target" == "user" ]; then
    compile_client
    compile_server
    compile_pbs
    compile_rhcr
fi

if [ "$target" == "clean" ]; then
    echo "Cleaning up..."
    rm -rf $current_path/client/build
    rm -rf $current_path/server/build
    rm -rf $current_path/planner/PBS/build
    rm -rf $current_path/planner/RHCR/build
    rm -rf $current_path/client/externalDependencies/rpclib/build
    echo "Cleanup complete."
fi