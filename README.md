# Lifelong MAPF Simulator Based on SMART

This is a lifelong MAPF simulator that considers differentiable drive robots with second order dynamics. It is built based on the [SMART simulator](https://jingtianyan.github.io/publication/2025-smart/).

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
    bash compile.sh rpc
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

## Running

#### Running without visualization
```bash
python run_lifelong.py maps/kiva_large_w_mode.json --headless True --screen 1 --num_agents 100 --save_stats True --rotation False --planner_invoke_policy default --sim_window_tick 10 --planner RHCR  --backup_solver PIBT --task_assigner_type windowed
```

This runs a simulation with 100 agents in the kiva map with the periodic planner invocation policy and windowed-multi-goal instance generator. The agent model is pebble motion. The planner is windowed PBS with planning window 1 second. The fail policy is PIBT.

To enable visualization, use `--headless=False`.