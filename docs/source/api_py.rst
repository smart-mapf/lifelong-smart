Quick Start with Python API
+++++++++++++++++++++++++++++++

We provide a Python API to easily run LSMART simulations with our provided planners, instance generators, invocation policies, and fail policies. You can customize the simulation parameters by changing the function arguments. In the repository, we have a python script ``run_lifelong.py`` that contains a function ``run_lifelong_argos`` to run LSMART simulations.

.. autofunction:: run_lifelong.run_lifelong_argos


Example Usage
+++++++++++++++++

LSMART has three explicit visualization modes. ``none`` runs without a user
interface, ``web`` streams events to the browser service, and ``argos`` opens
the native Qt/OpenGL ARGoS visualizer. The same modes are available from a
source build, Docker, and the Singularity image converted from Docker.

To run without visualization in the ``maps/kiva_large_w_mode.json`` map with
10 robots, ``RHCR`` planner (``PBS`` MAPF solver and ``SIPP`` single-agent
solver), ``PIBT`` fail policy, ``windowed`` problem instance generator, and
the default invocation policy, use:

.. code-block:: console

    python run_lifelong.py maps/kiva_large_w_mode.json --visualizer none --screen 0 --num_agents 10 --save_stats True --rotation False --planner_invoke_policy default --sim_window_tick 10 --planner RHCR  --backup_solver PIBT --task_assigner_type windowed


To run the same simulation with the native ARGoS visualizer:

.. code-block:: console

    python run_lifelong.py maps/kiva_large_w_mode.json --visualizer argos --screen 0 --num_agents 10 --save_stats True --rotation False --planner_invoke_policy default --sim_window_tick 10 --planner RHCR  --backup_solver PIBT --task_assigner_type windowed

The ``web`` mode is launched through the Bun service, which supplies the
external event-stream connection and serves the browser application. From a
source build:

.. code-block:: console

    cd web/lsmart-service
    bun run start --num_agents=10 --planner=RHCR --seed=42

Open ``http://localhost:3000`` and press **Start Simulation**. See
:doc:`install` for the equivalent Docker and Singularity commands, X11 setup
for the native visualizer, and reproducibility guidance.
