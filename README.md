# Lifelong MAPF Simulator Based on SMART

This is a lifelong MAPF simulator that considers differentiable drive robots with second order dynamics. It is built based on the [SMART simulator](https://jingtianyan.github.io/publication/2025-smart/).

## Installation

1.  Install RPC
    This repo requires [RPC](https://github.com/rpclib/rpclib) for communication
    between server and clients.
    Please install rpc using:
    ```bash
    bash compile.sh rpc
    ```

1.  Install Argos 3. Please refer to this [Link](https://www.argos-sim.info/core.php) for instruction.

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

    You can verify the correctness of the compilation by running: 
    ```bash
    argos3 -c experiments/diffusion_1.argos
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

Alternatively, you may compile rpc, server, client, and MAPF planner using:

```bash
bash compile.sh all
```

While developing, you may compile server, client, and MAPF planner using:

```bash
bash compile.sh user
```

## Running

#### Running with visualization
```
python run_lifelong.py
```

#### Running in headless mode
```
python run_lifelong.py --headless=True
```