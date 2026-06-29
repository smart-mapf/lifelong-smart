
## Installation
1.  Install Argos 3. Please refer to this [Link](https://www.argos-sim.info/core.php) for instruction.

    You can verify the correctness of the compilation by running:
    ```bash
    argos3 --version
    ```

1.  Install RPC
    This repo requires [RPC](https://github.com/rpclib/rpclib) for communication
    between server and clients.
    Please install rpc using:
    ```bash
    bash compile.sh rpclib
    ```

1.  Install the minimal Python dependencies for `run_lifelong.py`.
    ```bash
    python -m pip install -r requirement.txt
    ```

1.  Compile client.
    ```bash
    bash compile.sh client
    ```

    To produce debuggable code (slow), type:

    ```bash
    cd client
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    make
    cd ..
    ```


1.  Compile server.
    ```bash
    bash compile.sh server
    ```

1.  Compile MAPF planner. For now we support PBS and RHCR.
    ```bash
    bash compile.sh pbs
    bash compile.sh rhcr
    ```

Alternatively, you may compile rpc, server, client, and MAPF planners using:

```bash
bash compile.sh all
```

While developing, you may compile server, client, and MAPF planners using:

```bash
bash compile.sh user
```

## Container

You can also build and run LSMART inside a [Singularity](https://github.com/sylabs/singularity) container.

The container build expects **Argos3** to already exist in the repository
root because `singularity/container.def` copies it into the image:

1. `argos3_simulator-3.0.0-x86_64-beta59.deb`: Download it from [Argos3](https://www.argos-sim.info/core.php).

2. Install CPLEX at `CPLEX_Studio2210/`: If it exists in the repository root, the container
   build also copies it into the image and compiles the MASS planner. If it is missing, the
   container still builds, but skips MASS.

Build the container with:

```bash
bash singularity/build_container.sh
```

This produces `singularity/container.sif`. The build script first creates a
writable sandbox, runs it once to compile LSMART inside the container, and
then packs the result into the final `.sif` image. During the build, the
container also installs the Python packages from `requirement.txt`.

To run a headless simulation from the container:

```bash
singularity exec --cleanenv \
  singularity/container.sif \
  python run_lifelong.py maps/kiva_large_w_mode.json --headless True --screen 0 --num_agents 10 --save_stats True --rotation False --planner_invoke_policy default --sim_window_tick 10 --planner RHCR --backup_solver PIBT --task_assigner_type windowed --container True
```
