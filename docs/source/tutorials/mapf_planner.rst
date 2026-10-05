MAPF Planner Integration
+++++++++++++++++++++++++++++++

.. figure:: ../readme_assets/l-smart-2.0.png
   :alt: LSMART Pipeline
   :align: left
   :width: 100%


The MAPF planner takes in a MAPF problem instance with a time limit and returns collision-free paths within that time limit. The MAPF planner should return colliding paths in case of failure if the fail policy expects them.


Our Provided Planners
========================

We provide several built-in MAPF planners, including:

* **RHCR** (`Li et al. 2021`_): the Rolling Horizon Collision Resolution planner. RHCR plans for windowed paths for all robots. It supports planning with the pebble motion model and rotational motion model. RHCR is the default planner in LSMART.
* **MASS** (`Yan et al. 2025`_): the MAPF-SSIPP-SPS planner. MASS plans for full-horizon paths with 2nd order dynamics for all robots. MASS requires IBM CPLEX and is therefore not included in the published Docker image or the Docker-derived Singularity image. It is available in a source build configured with CPLEX.
* **PBS** (`Ma et al. 2019`_): the Priority-Based Search planner . PBS plans for full-horizon paths for all robots. It supports planning with the pebble motion model.
* **TPBS** (`Morag et al. 2025`_): the `Transient` Priority-Based Search planner . TPBS plans for full-horizon paths for all robots even if there are duplicate goals. It supports planning with the pebble motion model.

.. _Li et al. 2021: https://arxiv.org/abs/2005.07371
.. _Yan et al. 2025: https://arxiv.org/abs/2412.13359
.. _Ma et al. 2019: https://arxiv.org/abs/1812.06356
.. _Morag et al. 2025: https://ojs.aaai.org/index.php/SOCS/article/view/35998


Detailed usage of the built-in planners can be found in the :doc:`Quick Start with Python API <../api_py>` guide.

Add New Planners
========================

The MAPF planners use RPC to communicate with other modules in LSMART. Specifically, the planner shall implement an RPC client that connects to the RPC server in LSMART. The planner shall receive a MAPF problem instance and a time limit from LSMART, and return collision-free paths within that time limit.

Check LSMART Initialization and Invocation Status
-------------------------------------------------

The MAPF planner acts as an RPC client and polls LSMART's request/response
endpoints. Endpoint names are part of the wire protocol and must be used
exactly as shown below.

.. list-table:: Planner RPC protocol
   :header-rows: 1
   :widths: 22 18 60

   * - Endpoint
     - Result
     - Purpose
   * - ``is_initialized``
     - Boolean
     - Reports whether LSMART has initialized the simulation and ADG.
   * - ``invoke_planner``
     - Boolean
     - Reports whether LSMART is requesting a new plan.
   * - ``get_location``
     - JSON string
     - Returns the current MAPF problem instance.
   * - ``add_plan``
     - No value
     - Submits one JSON-encoded planning result to LSMART.

Before requesting an instance, call ``is_initialized`` and then
``invoke_planner``. These endpoints are implemented by the following server
handlers:

.. doxygenfunction:: rpc_api::isInitialized()

.. doxygenfunction:: rpc_api::invokePlanner()


Receive MAPF Problem Instances
---------------------------------

When ``invoke_planner`` returns true, call ``get_location`` to receive a MAPF
problem instance from LSMART. Its JSON schema is documented by the following
server handler:

.. doxygenfunction:: rpc_api::getRobotsLocation()

Return Plan Results
----------------------------

After planning, call ``add_plan`` with the JSON-encoded result whether planning
succeeded or failed. Its input schema is documented by the following server
handler:

.. doxygenfunction:: rpc_api::addNewPlan(string&)
