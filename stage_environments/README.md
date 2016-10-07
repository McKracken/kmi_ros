==================
stage_environments

Luca Iocchi, 2014
==================

This package contains a modified version of stage_ros with parametric names of topics and frames.

It also contains a Python script for dynamically creating worlds and another one for running
stage, robots and common nodes (amcl, move_base, glocalizer, gradient_based_navigation).

Quick use:

  $ cd scripts
  $ ./start_simulation.py
  -- play with the simulator --
  $ ./quit.sh

Command line use:
  $ ./start_simulation.py  <map_name> <robot_name> <localization> <navigation>
  
  e.g., 
  $ ./start_simulation.py dis_B1 diago amcl move_base



