Quick Start with Python API
+++++++++++++++++++++++++++++++

We provide a Python API to easily run LSMART simulations with our provided planners, instance generators, invocation policies, and fail policies. You can customize the simulation parameters by changing the function arguments. In the repository, we have a python script `run_lifelong.py` that contains a function `run_lifelong_argos` to run LSMART simulations.

.. autofunction:: run_lifelong.run_lifelong_argos


Example Usage
+++++++++++++++++

To run with visualization

.. code-block:: console

    python run_lifelong.py


To run in headless mode

.. code-block:: console

    python run_lifelong.py --headless=True