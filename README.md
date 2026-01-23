# LSMART

**L**ifelong **S**calable **M**ulti-**A**gent **R**ealistic **T**estbed (**LSMART**), a lifelong MAPF simulator that considers differentiable drive robots with second order dynamics. LSMART is built based on the [SMART simulator](https://github.com/smart-mapf/smart).

## Overview

![LSMART Pipeline](readme_assets/l-smart-2.0.png)

LSMART is an open-source simulator to evaluate any Multi-Agent Path Finding (MAPF) algorithm in a Fleet Management System (FMS) with Automated Guided Vehicles (AGVs). MAPF aims to move a group of agents from their corresponding starting locations to their goals. Lifelong MAPF (LMAPF) is a variant of MAPF that continuously assigns new goals for agents to reach. LMAPF applications, such as autonomous warehouses, often require a centralized, lifelong system to coordinate the movement of a fleet of robots, typically AGVs. However, existing works on MAPF and LMAPF often assume simplified kinodynamic models, such as pebble motion, as well as perfect execution and communication for AGVs. LSMART encapsulates key design choices of a real-world FMS in separate modules. These include:

1. **an invocation policy**: given the current states of the robots, it determines if the planner should be invoked.
2. **a MAPF problem instance generator**: given the current states of the robots, it returns the a MAPF problem instance.
3. **a MAPF planner**: given a MAPF problem instance with start and goal locations, it returns collision-free paths.
4. **a fail policy**: if the MAPF planner fails to return collision-free paths, it recovers the system from failure.


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

## Usage

#### Running with visualization
```
python run_lifelong.py
```

#### Running in headless mode
```
python run_lifelong.py --headless=True
```

## Documentation

The documentation is available online [here](). We suggest that new users start with the [tutorials]().

## Paper and Citation

We have a white paper describing LSMART in detail and presenting key experimental results of our implemented solutions to different modules of LSMART, see [here](). If you use LSMART for your research, cite the following:
```bibtex
@article{YanAndZhang2026LSMART,
    author    = {Jingtian Yan, Yulun Zhang, Zhenting Liu, Han Zhang, He Jiang, Jingkai Chen, Stephen F. Smith and Jiaoyang Li},
    title     = {Lifelong Scalable Multi-Agent Realistic Testbed and A Comprehensive Study on Design Choices in Lifelong AGV Fleet Management Systems},
    journal   = {ArXiv},
    volume    = {},
    year      = {2026}
}
```