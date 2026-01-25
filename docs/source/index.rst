.. LSMART documentation master file.

.. raw:: html

   <div class="lsmart-hero lsmart-hero--home">
     <video class="lsmart-hero-video" autoplay muted loop playsinline preload="auto">
       <source src="_static/front-video.mp4" type="video/mp4">
     </video>

     <div class="lsmart-hero-content">
       <div>
         <img class="lsmart-hero-logo" src="_static/lsmart-logo-black.png" alt="LSMART logo">
         <div class="lsmart-hero-buttons">
           <a class="lsmart-btn lsmart-btn-primary" href="getting_started.html">Install</a>
           <a class="lsmart-btn" href="api.html">Documentation</a>
           <a class="lsmart-btn" href="https://github.com/lunjohnzhang/lifelong_mapf_argos">GitHub</a>
         </div>
       </div>
     </div>
   </div>

.. raw:: html

   <div class="lsmart-home-content">

**Lifelong Scalable Multi-Agent Realistic Testbed (LSMART)**
============================================================

.. image:: _static/l-smart-2.0.png
   :alt: LSMART Pipeline
   :align: center
   :width: 100%

Introduction
+++++++++++++++++

LSMART is an open-source simulator to evaluate any Multi-Agent Path Finding (MAPF) algorithm in a Fleet Management System (FMS) with Automated Guided Vehicles (AGVs) under lifelong MAPF (LMAPF) settings. MAPF aims to move a group of agents from their corresponding starting locations to their goals. Lifelong MAPF (LMAPF) is a variant of MAPF that continuously assigns new goals for agents to reach. LMAPF applications, such as autonomous warehouses, often require a centralized, lifelong system to coordinate the movement of a fleet of robots, typically AGVs. However, existing works on MAPF and LMAPF often assume simplified kinodynamic models, such as pebble motion, as well as perfect execution and communication for AGVs. LSMART encapsulates key design choices of a real-world FMS in separate modules. These include:

1. **an invocation policy**: given the current states of the robots, it determines if the planner should be invoked.
2. **a MAPF problem instance generator**: given the current states of the robots, it returns the a MAPF problem instance.
3. **a MAPF planner**: given a MAPF problem instance with start and goal locations, it returns collision-free paths.
4. **a fail policy**: if the MAPF planner fails to return collision-free paths, it recovers the system from failure.


Real-world Considerations
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.. raw:: html

   <div class="lsmart-video-row">

     <figure class="lsmart-video-figure-two">
       <video class="lsmart-differentiable-drive-video" autoplay muted loop playsinline preload="auto">
         <source src="_static/differentiable-drive.mp4" type="video/mp4">
       </video>
       <figcaption>
         Differentiable Drive Robot Kinodynamics
       </figcaption>
     </figure>

     <figure class="lsmart-video-figure-two">
       <video class="lsmart-adg-video" autoplay muted loop playsinline preload="auto">
         <source src="_static/adg-in-play.mp4" type="video/mp4">
       </video>
       <figcaption>
         Real-world Execution Uncertainties
       </figcaption>
     </figure>

   </div>

We consider realistic AGV kinodynamics by modeling them as differentiable drive robots, which can move forward and rotate in place with constraints in maximum velocity and acceleration. This is in contrast to prior works that model AGVs as simple omnidirectional/pebble motion agents. We also simulate real-world execution delays and use Action Dependency Graph (ADG) [1] to ensure collision-free of the executed paths.


Insane Scalability
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.. raw:: html

    <figure class="lsmart-video-figure-one">
    <video
        class="lsmart-scalability-video"
        autoplay
        muted
        loop
        playsinline
        preload="auto">
        <source src="_static/warehouse-large.mp4" type="video/mp4">
    </video>
    <figcaption>
        Simulation of 1000 robots in a large warehouse.
    </figcaption>
    </figure>

LSMART is highly scalable in terms of the number of robots and the size of the map. Here we show a simulation of 1000 robots in the *warehouse-10-20-10-2-1* map from the MAPF benchmark [2]. We use PIBT [3] as the underlying MAPF planner and plan windowed paths every 1 simulation second. The simulation runs for 600 simulation seconds and takes 268 seconds to finish with a AMD Ryzen 9 9950X 16-Core Processor.

**References**
++++++++++++++++++++++++++++++

[1] Hönig, W.; Kiesel, S.; Tinka, A.; Durham, J. W.; and Ayanian, N. 2019. Persistent and Robust Execution of MAPF Schedules in Warehouses. IEEE Robotics and Automation Letters, 4: 1125-1131.

[2] Stern, R.; Sturtevant, N. R.; Felner, A.; Koenig, S.; Ma, H.; Walker, T. T.; Li, J.; Atzmon, D.; Cohen, L.; Kumar, T. K. S.; Barták, R.; and Boyarski, E. 2019. Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks. In Proceedings of the International Symposium on Combinatorial Search (SoCS), 151-159.

[3] Okumura, K.; Machida, M.; Défago, X.; and Tamura, Y. 2019. Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding. In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), 535-542.

**Paper and Citation**
======================
.. code-block:: bibtex

    @article{YanAndZhang2026LSMART,
        author    = {Jingtian Yan, Yulun Zhang, Zhenting Liu, Han Zhang, He Jiang, Jingkai Chen, Stephen F. Smith and Jiaoyang Li},
        title     = {Lifelong Scalable Multi-Agent Realistic Testbed and A Comprehensive Study on Design Choices in Lifelong AGV Fleet Management Systems},
        journal   = {ArXiv},
        volume    = {},
        year      = {2026}
    }

.. raw:: html
   </div> <!-- .lsmart-home-content -->

.. toctree::
    :hidden:
    :maxdepth: 1
    :caption: API Reference

    api