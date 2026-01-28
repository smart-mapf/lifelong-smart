import os
import sys
import pathlib
import subprocess
import time
import fire
import numpy as np
import logging
import datetime

from typing import List, Tuple
from lifelong_mapf_argos.ArgosConfig import (SERVER_EXE, PBS_EXE, TPBS_EXE,
                                             RHCR_EXE, MASS_EXE,
                                             CONTAINER_PROJECT_ROOT,
                                             PROJECT_ROOT, setup_logging)
from lifelong_mapf_argos.ArgosConfig.ToArgos import (obstacles, parse_map_file,
                                                     create_Argos)

logger = logging.getLogger(__name__)


def get_current_time() -> str:
    """Get the current time in the format YYYY-MM-DD HH-MM-SS.FFF."""
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def init_start_locations(
    map_str: List[str],
    num_agents: int,
) -> List[Tuple[str, str]]:
    # Get free locations
    h, w = len(map_str), len(map_str[0])
    free_locations = []
    for i in range(h):
        for j in range(w):
            if map_str[i][j] not in obstacles:
                free_locations.append(i * w + j)
    if len(free_locations) < num_agents:
        logger.error(
            f"Number of agents ({num_agents}) exceeds number of free locations ({len(free_locations)})."
        )
        exit(-1)
    # Randomly select start locations
    starts = np.random.choice(free_locations, size=num_agents, replace=False)
    # We need to convert the coordinate from (row, col) to (col, row) because
    # this is how Argos expects it.
    return [(str(start % w), str(start // w)) for start in starts]


def check_file(file_path: str):
    if not os.path.exists(file_path):
        logger.info(f"{file_path} not exists!")
        return False
    return True


def run_simulator(args, timeout: float = None, output_log: str = None):
    server_command, client_command, planner_command = args
    f = open(output_log, 'w') if output_log else None

    # Start the server process
    server_process = subprocess.Popen(server_command, stdout=f, stderr=f)

    # Wait for a short period to ensure the server has started
    time.sleep(1)

    # Start the client process
    client_process = subprocess.Popen(client_command, stdout=f, stderr=f)

    # Wait for the client process to complete
    # time.sleep(5)
    planner_process = subprocess.Popen(planner_command, stdout=f, stderr=f)

    # The client process will call the server to end, then the client end. The
    # planner will detect the end of the server and end itself.
    try:
        client_process.wait(timeout=timeout)
        print(f"[{get_current_time()}] [Py] Client process finished.", file=f)

        server_process.wait(timeout=timeout)
        # planner_process.wait()
        print(f"[{get_current_time()}] [Py] Server process finished.", file=f)

        planner_process.kill()
        print(f"[{get_current_time()}] [Py] Planner process finished.", file=f)
    except subprocess.TimeoutExpired:
        # print("Timeout expired, killing processes...")
        logger.info("Timeout expired, killing processes...")
        client_process.kill()
        server_process.kill()
        planner_process.kill()
    finally:
        if f:
            f.close()
        # print("Processes killed.")
        logger.info("Processes killed.")


def run_lifelong_argos(
    map_filepath: str = "maps/kiva_large_w_mode.json",
    num_agents: int = 100,
    headless: bool = False,
    argos_config_filepath: str = "output.argos",
    stats_name: str = "stats.json",
    save_stats: bool = False,
    output_log: str = None,
    port_num: int = 8182,
    n_threads: int = 1,
    sim_duration: int = 600 * 10,
    sim_window_tick: int = 20,
    ticks_per_second: int = 10,
    velocity: float = 200.0,
    planner: str = "RHCR",
    container: bool = False,
    seed: int = 42,
    screen: int = 0,
    backup_solver: str = "PIBT",
    planner_invoke_policy: str = "default",
    task_assigner_type: str = "windowed",
    planning_window: int = 10,
    frame_grab: bool = False,
    cutoffTime: int = 1,
    # RHCR parameters
    solver: str = "PBS",
    single_agent_solver: str = "SIPP",
    rotation: bool = False,
    rotation_time: int = 1,
    prioritize_start: bool = True,
    suboptimal_bound: float = 1,
    log: bool = False,
    save_result: bool = False,
    save_solver: bool = False,
    save_heuristics_table: bool = False,
):
    """Function to run the LSMART simulator with the given parameters.

    Args:
        map_filepath (str, optional): file path to map. Example maps are in the ``maps`` directory. If maps contains workstations (``w``) and endpoints (``e``), robots' tasks will be assigned alternately between workstations and endpoints. If not, robots' tasks will be randomly generated from the empty spaces. Defaults to ``maps/kiva_large_w_mode.json``.
        num_agents (int, optional): number of robots. Defaults to 32.
        headless (bool, optional): whether run in headless mode. If False, a visualization will be generated. Defaults to False.
        argos_config_filepath (str, optional): file path to write the generated
            Argos config file. Defaults to "output.argos".
        stats_name (str, optional): file path to store the stats from the
            simulator. Defaults to "stats.json".
        save_stats (bool , optional): whether to save the stats. Defaults to
            False.
        output_log (str, optional): file path to store the stdout from the
            simulator. If None, the output will not be saved to a file.
            Defaults to None.
        port_num (int, optional): port number of RPC server. Defaults to 8182.
        n_threads (int, optional): number of threads to run Argos. Defaults to 1.
        ticks_per_second (int, optional): the simulator runs in ``ticks``. The states of the robots are updated per tick. ``ticks_per_second`` specifies the number of ticks per simulation second used by the simulator. Defaults to 10.
        sim_duration (int, optional): number of simulation ticks to run the
            simulator. Defaults to 1800 * 10, meaning 1800 seconds.
        sim_window_tick (int, optional): number of ticks to invoke the planner. Only applies to the periodic invocation policy. Defaults to 20 ticks.
        velocity (float, optional): velocity of the robots in cm/s. Defaults to
            200.0 cm/s.
        planner (str, optional): planner to use. Options include:

            - ``RHCR``: the Rolling Horizon Collision Resolution planner, (`Li et al. 2021`_). RHCR plans for windowed paths for all robots.
            - ``MASS``: the MAPF-SSIPP-SPS planner, (`Yan et al. 2025`_). MASS plans for full-horizon paths with 2nd order dynamics for all robots.
            - ``PBS``: the Priority-Based Search planner (`Ma et al. 2019`_). PBS plans for full-horizon paths for all robots.
            - ``TPBS``: the `Transient` Priority-Based Search planner (`Morag et al. 2025`_). TPBS plans for full-horizon paths for all robots even if there are duplicate goals.

            Defaults to ``RHCR``.
        container (bool, optional): whether to run in a `singularity`_ container. Defaults to False.
        seed (int, optional): random seed. Defaults to 42.
        screen (int, optional): logging options. Defaults to 0.
        backup_solver (str, optional): backup solver (fail policy) used in case the MAPF planner fails. Options include:

            - ``PIBT``: the Priority Inheritance with Backtracking, (`Okumura et al. 2019`_).
            - ``LRA``: the Local Repair Guided Waits, (`Li et al. 2021`_).
            - ``GuidedPIBT``: Guided PIBT, (`Chen et al. 2024`_).

            Defaults to ``PIBT``.
        planner_invoke_policy (str, optional): planner invocation policy, options include:

            - ``default``: the periodic policy where the planner is invoked periodically every ``sim_window_tick`` ticks.
            - ``no_action``: the event-based policy where the planner is invoked when at least one robot has no action to execute in the ADG (`Hönig et al. 2019`_).

            Defaults to ``default``.
        task_assigner_type (str, optional): task assigner (MAPF problem instance generator) used to generate problem instances. Options include:

            - ``windowed``: the windowed task assigner (`Li et al. 2021`_), which assigns tasks within the planning window. This can only be used with the ``RHCR`` planner.
            - ``distinct-one-goal``: the distinct one-goal task assigner, which assigns each robot a distinct goal. This can only be used with the ``PBS`` and ``MASS`` planners.
            - ``one-goal``: the one-goal task assigner, which assigns each robot a goal regardless of duplicates. This can only be used with the ``TPBS`` planner.

            Defaults to ``windowed``.
        planning_window (int, optional): planning window in timesteps. The final planning window is the max of this value and the inferred planning window from sim_window_tick.

            Specifically, given ``sim_window_tick`` and ``ticks_per_second``, we compute the minimal planning window in timesteps required to cover the simulation window as follows:

            .. math::
                planning\_window\_ts = \\lceil \\frac{sim\_window\_tick}{ticks\_per\_second} \\times \\frac{velocity}{100} \\rceil

            Then given the user-specified ``planning_window``, the final planning window is:

            .. math::
                planning\_window = max(planning\_window\_ts, planning\_window)

            Defaults to 10.

        cutoffTime (int, optional): time limit of the planner in seconds. With the periodic invocation policy (``default``), the ``cutoffTime`` should be less than or equal to the simulation window in seconds (:math:`\\frac{sim\_window\_tick}{ticks\_per\_second}`). Defaults to 1.
        frame_grab (bool, optional): whether to enable frame grabber in Argos.
            If enabled, the simulator will save screenshots to the ``frames``
            folder. The screenshots can be combined into a video using external
            tools. Defaults to False.
        solver (str, optional): MAPF solver in RHCR. Options include ``PBS`` (`Ma et al. 2019`_) and ``PIBT`` (`Okumura et al. 2019`_). Defaults to ``PBS``.
        single_agent_solver (str, optional): single agent solver in RHCR.
            Options include ``SIPP`` (`Phillips et al. 2011`_) and ``ASTAR``.
            Defaults to ``SIPP``.
        rotation (bool, optional): whether the single-agent planning consider rotation in RHCR. Defaults to False.
        rotation_time (int, optional): rotation time in timesteps in RHCR.
            Defaults to 1.
        prioritize_start (bool, optional): prioritize start in RHCR. Defaults
            to True.
        suboptimal_bound (float, optional): suboptimality bound of certain
            solvers in RHCR. Defaults to 1.
        log (bool, optional): logging in RHCR. Defaults to False.
        save_result (bool, optional): save planning results or not in RHCR.
            Defaults to False.
        save_solver (bool, optional): save MAPF solver process in RHCR or not.
            Defaults to False.
        save_heuristics_table (bool, optional): save heuristic table or not.
            Defaults to False.

    .. _Li et al. 2021: https://arxiv.org/abs/2005.07371
    .. _Yan et al. 2025: https://arxiv.org/abs/2412.13359
    .. _Ma et al. 2019: https://arxiv.org/abs/1812.06356
    .. _Morag et al. 2025: https://ojs.aaai.org/index.php/SOCS/article/view/35998
    .. _Okumura et al. 2019: https://arxiv.org/abs/1901.11282
    .. _Chen et al. 2024: https://arxiv.org/abs/2308.11234
    .. _Hönig et al. 2019: https://ieeexplore.ieee.org/document/8620328
    .. _Phillips et al. 2011: https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf
    .. _singularity: https://sylabs.io/singularity/
    """
    np.random.seed(seed)
    setup_logging()
    map_data, width, height = parse_map_file(map_filepath)

    # Transform the map and scen to Argos config file, obstacles: '@', 'T'
    robot_init_pos = init_start_locations(map_data, num_agents)

    # Use absolute path for argos config
    argos_config_filepath = os.path.abspath(argos_config_filepath)

    create_Argos(
        map_data=map_data,
        output_file_path=argos_config_filepath,
        width=width,
        height=height,
        robot_init_pos=robot_init_pos,
        curr_num_agent=num_agents,
        port_num=port_num,
        n_threads=n_threads,
        visualization=not headless,
        sim_duration=sim_duration,
        ticks_per_second=ticks_per_second,
        screen=screen,
        velocity=velocity,
        container=container,
        seed=seed,
        frame_grab=frame_grab,
    )
    if screen > 0:
        logger.info(f"Argos config file created at {argos_config_filepath}.")

    # Infer look_ahead_dist from cutoffTime
    # The look_ahead_dist is the number of actions the planner should
    # look ahead to find the start location for the robot. The planner will
    # return in cutoffTime seconds, so look_ahead_dist should be set to the
    # number of actions that can be executed in cutoffTime seconds.
    # Each action corresponds to half of a grid, which is 0.5 m, so the
    # shortest time it takes to execute an action is
    # 0.5 m / (velocity / 100 m/s).
    look_ahead_dist = np.ceil(cutoffTime / (0.5 /
                                            (velocity / 100))).astype(int)
    look_ahead_tick = np.ceil(look_ahead_dist * (0.5 / (velocity / 100)) *
                              ticks_per_second).astype(int)

    # Infer the planning window
    # planning window = sim window in timesteps + look_ahead_dist in timesteps
    # The planner should make sure that the plan is valid for the next
    # simulation window + look_ahead_dist
    # Convert sim_window from ticks to timesteps, and velocity from cm/s to m/s
    # plan_window_ts = np.ceil((sim_window_tick / ticks_per_second) *
    #                          (velocity / 100) +
    #                          look_ahead_dist / 2).astype(int)
    plan_window_ts = np.ceil(
        (sim_window_tick / ticks_per_second) * (velocity / 100)).astype(int)
    # We need a planning window in all cases because the backup planner might
    # be windowed
    plan_window_ts = np.max([plan_window_ts, planning_window])
    logger.info(f"{planner}: Planning window in timesteps: {plan_window_ts}")

    # Path to the executables
    if container:
        server_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / SERVER_EXE
        pbs_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / PBS_EXE
        tpbs_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / TPBS_EXE
        rhcr_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / RHCR_EXE
        mass_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / MASS_EXE
    else:
        server_path = pathlib.Path(PROJECT_ROOT) / SERVER_EXE
        pbs_path = pathlib.Path(PROJECT_ROOT) / PBS_EXE
        tpbs_path = pathlib.Path(PROJECT_ROOT) / TPBS_EXE
        rhcr_path = pathlib.Path(PROJECT_ROOT) / RHCR_EXE
        mass_path = pathlib.Path(PROJECT_ROOT) / MASS_EXE

    try:
        # print("Running simulator ...")
        logger.info(
            f"Running simulator with {num_agents} agents at port {port_num} ..."
        )
        server_command = [
            str(server_path),
            f"--num_robots={num_agents}",
            f"--port_number={port_num}",
            f"--output_file={stats_name}",
            f"--save_stats={str(save_stats).lower()}",
            f"--screen={screen}",
            f"--total_sim_step_tick={sim_duration}",
            f"--ticks_per_second={ticks_per_second}",
            f"--look_ahead_dist={look_ahead_dist}",
            f"--look_ahead_tick={look_ahead_tick}",
            f"--seed={seed}",
            f"--sim_window_tick={sim_window_tick}",
            f"--sim_window_timestep={plan_window_ts}",
            f"--plan_window_timestep={plan_window_ts}",
            f"--planner_invoke_policy={planner_invoke_policy}",
            f"--backup_planner={backup_solver}",
            f"--backup_single_agent_solver={single_agent_solver}",
            f"--map={map_filepath}",
            f"--grid_type=regular",
            f"--rotation={str(rotation).lower()}",
            f"--rotation_time={rotation_time}",
            f"--save_heuristics_table={str(save_heuristics_table).lower()}",
            f"--task_assigner_type={task_assigner_type}",
        ]
        client_command = [
            "argos3",
            "-c",
            f"{argos_config_filepath}",
            # No color displays better in file output
            "--no-color",
        ]

        # Standard MAPF PBS
        if planner == "PBS":
            planner_command = [
                pbs_path,
                f"--map={map_filepath}",
                f"--agentNum={num_agents}",
                f"--portNum={port_num}",
                f"--seed={seed}",
                f"--screen={screen}",
                f"--cutoffTime={cutoffTime}",
            ]
        # Transient MAPF PBS
        elif planner == "TPBS":
            planner_command = [
                tpbs_path,
                f"--map={map_filepath}",
                f"--agentNum={num_agents}",
                f"--portNum={port_num}",
                f"--seed={seed}",
                f"--screen={screen}",
                f"--cutoffTime={cutoffTime}",
            ]
        # RHCR, typically windowed PBS
        elif planner == "RHCR":
            planner_command = [
                rhcr_path,
                f"--map={map_filepath}",
                f"--agentNum={num_agents}",
                f"--port_number={port_num}",
                f"--seed={seed}",
                f"--screen={screen}",
                f"--planning_window={plan_window_ts}",
                f"--simulation_window={plan_window_ts}",
                f"--scenario=SMART",
                f"--cutoffTime={cutoffTime}",
                f"--rotation={rotation}",
                f"--solver={solver}",
                f"--single_agent_solver={single_agent_solver}",
                f"--backup_solver={backup_solver}",
                f"--rotation_time={rotation_time}",
                f"--prioritize_start={str(prioritize_start).lower()}",
                f"--suboptimal_bound={suboptimal_bound}",
                f"--log={str(log).lower()}",
                f"--save_result={str(save_result).lower()}",
                f"--save_solver={str(save_solver).lower()}",
                f"--save_heuristics_table={str(save_heuristics_table).lower()}",
            ]
        # MASS, typically PBS with 2nd order dynamics
        elif planner == "MASS":
            planner_command = [
                mass_path,
                f"--map={map_filepath}",
                f"--agentNum={num_agents}",
                f"--portNum={port_num}",
                f"--seed={seed}",
                f"--screen={screen}",
                f"--simulation_window={sim_window_tick / ticks_per_second}",
                f"--cutoffTime={cutoffTime}",
                f"--saveInstance={False}",
            ]
        else:
            logger.error(f"Unknown planner: {planner}")
            exit(-1)
        run_simulator(
            args=(server_command, client_command, planner_command),
            timeout=20 * sim_duration / ticks_per_second,
            output_log=output_log,
        )
    except KeyboardInterrupt:
        # print("KeyboardInterrupt: Stopping the experiment ...")
        logger.info("KeyboardInterrupt: Stopping the experiment ...")


if __name__ == "__main__":
    fire.Fire(run_lifelong_argos)
