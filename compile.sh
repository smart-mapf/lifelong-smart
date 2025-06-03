# Compile everything in the project
target="$1"
current_path=$(pwd)

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

if [ "$target" == "all" ]; then
    compile_rpclib
    compile_client
    compile_server
    compile_pbs
fi