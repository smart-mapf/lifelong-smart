# Compile everything in the project
target="$1"
shift 1
CPLEX_DIR="${CPLEX_DIR:-}"

while getopts "c:" flag; do
    case "$flag" in
        c) CPLEX_DIR=$OPTARG;;
        *) echo "Invalid option. ${USAGE}"
    esac
done

echo "In lifelong argos: Using CPLEX directory: ${CPLEX_DIR}"

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
}


compile_tpbs() {
    echo "Compiling Transient PBS..."
    cd $current_path/planner/Transient_PBS
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j $cpuCores
}

compile_rhcr() {
    echo "Compiling RHCR..."
    cd $current_path/planner/RHCR
    rm -rf build
    bash compile.sh
}

compile_mass() {
    echo "Compiling MASS..."
    mass_cplex_dir="${CPLEX_DIR}"
    if [ -z "${mass_cplex_dir}" ]; then
        mass_cplex_dir="$current_path/CPLEX_Studio2210"
    fi
    # ponytail: MASS depends on CPLEX; skip it when the SDK is absent.
    if [ ! -d "${mass_cplex_dir}" ]; then
        echo "Skipping MASS because no CPLEX directory was found at ${mass_cplex_dir}."
        return 0
    fi
    cd $current_path/planner/MASS
    rm -rf build
    bash compile.sh -c "${mass_cplex_dir}"
}

compile_extviz() {
    echo "Compiling external visualizer plugin..."
    cd $current_path/plugins/visualizers/external_visualizer
    rm -rf build
    mkdir build
    cd build
    cmake ..
    make -j $cpuCores
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

if [ "$target" == "tpbs" ]; then
    compile_tpbs
fi

if [ "$target" == "rhcr" ]; then
    compile_rhcr
fi

if [ "$target" == "mass" ]; then
    compile_mass
fi

if [ "$target" == "extviz" ]; then
    compile_extviz
fi

if [ "$target" == "all" ]; then
    compile_rpclib
    compile_mass
    compile_client
    compile_server
    compile_pbs
    compile_tpbs
    compile_rhcr
fi

if [ "$target" == "user" ]; then
    compile_client
    compile_server
    compile_pbs
    compile_tpbs
    compile_rhcr
    compile_mass
fi

if [ "$target" == "clean" ]; then
    echo "Cleaning up..."
    rm -rf $current_path/client/build
    rm -rf $current_path/server/build
    rm -rf $current_path/planner/PBS/build
    rm -rf $current_path/planner/Transient_PBS/build
    rm -rf $current_path/planner/RHCR/build
    rm -rf $current_path/planner/MASS/build
    rm -rf $current_path/client/externalDependencies/rpclib/build
    echo "Cleanup complete."
fi
