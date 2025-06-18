import os

_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.dirname(_current_dir)
CONTAINER_PROJECT_ROOT = "/usr/project/lifelong_mapf_argos"
PROJECT_ROOT = _parent_dir
SERVER_EXE = "server/build/ADG_server"
PBS_EXE = "planner/PBS/build/pbs"
RHCR_EXE = "planner/RHCR/build/lifelong"
FOOTBOT_DIFFUSION_CONTROLLER_LIB = "client/build/controllers/footbot_diffusion/libfootbot_diffusion"
TRAJECTORY_LOOP_FUNCTIONS_LIB = "client/build/loop_functions/trajectory_loop_functions/libtrajectory_loop_functions"
