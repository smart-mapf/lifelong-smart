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
from lifelong_mapf_argos.ArgosConfig import (SERVER_EXE, PBS_EXE, RHCR_EXE,
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
    grid_type: str = "regular",
) -> List[Tuple[str, str]]:
    if grid_type == "regular":
        non_starts = obstacles
    elif grid_type == "one_bot_per_aisle":
        non_starts = obstacles + ["e"]
    # Get free locations
    h, w = len(map_str), len(map_str[0])
    free_locations = []
    for i in range(h):
        for j in range(w):
            if map_str[i][j] not in non_starts:
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
    num_agents: int = 50,
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
    look_ahead_dist: int = 10,
    planner: str = "RHCR",  # ["PBS", "RHCR"]
    container: bool = False,
    seed: int = 42,
    screen: int = 0,
    # RHCR parameters
    planning_window: int = 10,
    scenario: str = "SMART",
    task: str = "",
    cutoffTime: int = 1,
    id: bool = False,
    solver: str = "PBS",
    single_agent_solver: str = "SIPP",
    backup_solver: str = "PIBT",  # ["PIBT", "LRA"]
    lazyP: bool = False,
    travel_time_window: int = 0,
    potential_function: str = "NONE",
    potential_threshold: int = 0,
    rotation: bool = False,
    rotation_time: int = 1,
    robust: int = 0,
    CAT: bool = False,
    hold_endpoints: bool = False,
    dummy_paths: bool = False,
    prioritize_start: bool = True,
    suboptimal_bound: float = 1,
    log: bool = False,
    save_result: bool = False,
    save_solver: bool = False,
    save_heuristics_table: bool = False,
    left_w_weight: float = 1.0,
    right_w_weight: float = 1.0,
    grid_type: str = "regular",
):
    """Function to run the lifelong SMART simulator with the given parameters.

    Args:
        map_filepath (str, optional): file path to map..
        num_agents (int, optional): number of robots. Defaults to 32.
        headless (bool, optional): whether run with visualization. Defaults to
            False.
        argos_config_filepath (str, optional): file path to write the generated
            Argos config file. Defaults to "output.argos".
        stats_name (str, optional): file path to store the stats from the
            simulator. Defaults to "stats.csv".
        save_stats (bool , optional): whether to save the stats. Defaults to
            False.
        output_log (str, optional): file path to store the stdout from the
            simulator. If None, the output will not be saved to a file.
            Defaults to None
        port_num (int, optional): port number of RPC server. Defaults to 8182.
        n_threads (int, optional): number of threads to run Argos. Defaults to 1.
        sim_duration (int, optional): number of simulation ticks to run the
            simulator. Defaults to 1800*10.
        sim_window_tick (int, optional): number of ticks to invoke the planner.
        ticks_per_second (int, optional): number of updates (ticks) per
            simulation second
        velocity (float, optional): velocity of the robots in cm/s. Defaults to
            200.0 cm/s.
        look_ahead_dist (int, optional): look ahead distance for the planner to
            obtain robot goal location.
        planner (str, optional): planner to use. Defaults to "RHCR".
        container (bool, optional): whether to run in a container. Defaults to
            False.
        seed (int, optional): random seed. Defaults to 42.
        screen (int, optional): logging options. Defaults to 0.
    """
    np.random.seed(seed)
    setup_logging()
    # print(f"Map Name: {map_filepath}")
    map_data, width, height = parse_map_file(map_filepath)

    # Transform the map and scen to Argos config file, obstacles: '@', 'T'
    # if screen > 0:
    #     # print("Creating Argos config file ...")
    #     logger.info("Creating Argos config file ...")
    robot_init_pos = init_start_locations(map_data, num_agents, grid_type)

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
    )
    if screen > 0:
        # print("Argos config file created.")
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
    if planner in ["RHCR"]:
        plan_window_ts = np.max([plan_window_ts, planning_window])
    logger.info(f"Planning window in timesteps: {plan_window_ts}")
    # plan_window_ts = int(np.ceil(1.5 * look_ahead_dist * (velocity / 100)))

    # Path to the executables
    if container:
        server_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / SERVER_EXE
        pbs_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / PBS_EXE
        rhcr_path = pathlib.Path(CONTAINER_PROJECT_ROOT) / RHCR_EXE
    else:
        server_path = pathlib.Path(PROJECT_ROOT) / SERVER_EXE
        pbs_path = pathlib.Path(PROJECT_ROOT) / PBS_EXE
        rhcr_path = pathlib.Path(PROJECT_ROOT) / RHCR_EXE

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
        ]
        client_command = [
            "argos3",
            "-c",
            f"{argos_config_filepath}",
            # No color displays better in file output
            "--no-color",
        ]

        if planner == "PBS":
            planner_command = [
                pbs_path,
                f"--map={map_filepath}",
                f"--agentNum={num_agents}",
                f"--portNum={port_num}",
                f"--seed={seed}",
                f"--screen={screen}",
                f"--cutoffTime={cutoffTime}",
                f"--simulation_window={plan_window_ts}",
            ]
        elif planner == "RHCR":
            # if rotation:
            #     cutoffTime = int(cutoffTime * 10)
            planner_command = [
                rhcr_path,
                f"--map={map_filepath}",
                # f"--task={task}",
                f"--agentNum={num_agents}",
                f"--port_number={port_num}",
                f"--seed={seed}",
                f"--screen={screen}",
                f"--planning_window={plan_window_ts}",
                f"--simulation_window={plan_window_ts}",
                f"--scenario={scenario}",
                f"--cutoffTime={cutoffTime}",
                f"--rotation={rotation}",
                f"--solver={solver}",
                f"--id={str(id).lower()}",
                f"--single_agent_solver={single_agent_solver}",
                f"--backup_solver={backup_solver}",
                f"--lazyP={str(lazyP).lower()}",
                f"--travel_time_window={travel_time_window}",
                f"--potential_function={potential_function}",
                f"--potential_threshold={potential_threshold}",
                f"--rotation_time={rotation_time}",
                f"--robust={robust}",
                f"--CAT={str(CAT).lower()}",
                f"--hold_endpoints={str(hold_endpoints).lower()}",
                f"--dummy_paths={str(dummy_paths).lower()}",
                f"--prioritize_start={str(prioritize_start).lower()}",
                f"--suboptimal_bound={suboptimal_bound}",
                f"--log={str(log).lower()}",
                f"--save_result={str(save_result).lower()}",
                f"--save_solver={str(save_solver).lower()}",
                f"--save_heuristics_table={str(save_heuristics_table).lower()}",
                f"--left_w_weight={left_w_weight}",
                f"--right_w_weight={right_w_weight}",
                f"--grid_type={grid_type}",
            ]
            if task != "":
                planner_command.append(f"--task={task}")
        run_simulator(
            args=(server_command, client_command, planner_command),
            timeout=10 * sim_duration / ticks_per_second,
            output_log=output_log,
        )
    except KeyboardInterrupt:
        # print("KeyboardInterrupt: Stopping the experiment ...")
        logger.info("KeyboardInterrupt: Stopping the experiment ...")


if __name__ == "__main__":
    fire.Fire(run_lifelong_argos)
