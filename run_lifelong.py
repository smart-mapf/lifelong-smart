import os
import pathlib
import ArgosConfig
import subprocess
import time
import fire
import numpy as np

from typing import List, Tuple


def init_start_locations(
    map_str: List[str],
    num_agents: int,
) -> List[Tuple[str, str]]:
    # Get free locations
    h, w = len(map_str), len(map_str[0])
    free_locations = []
    for i in range(h):
        for j in range(w):
            if map_str[i][j] not in ArgosConfig.obstacles:
                free_locations.append(i * w + j)
    if len(free_locations) < num_agents:
        print(
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
        print(f"{file_path} not exists!")
        return False
    return True


def run_simulator(args):
    server_command, client_command, planner_command = args
    # Start the server process
    server_process = subprocess.Popen(server_command)

    # Wait for a short period to ensure the server has started
    time.sleep(3)
    # Start the client process
    os.chdir("./client")
    # os.chdir("client")
    client_process = subprocess.Popen(client_command)

    # Wait for the client process to complete
    # time.sleep(5)
    os.chdir("..")
    planner_process = subprocess.Popen(planner_command)

    # The client process will call the server to end, then the client end. The
    # planner will detect the end of the server and end itself.
    client_process.wait()
    server_process.wait()
    planner_process.wait()
    # planner_process.kill()


def main(
    map_filepath: str = "../maps/mapf_bench/no_guidance/random-32-32-20.json",
    num_agents: int = 32,
    headless: bool = False,
    argos_config_filepath: str = "output.argos",
    stats_name: str = "stats.csv",
    save_stats: bool = False,
    port_num: int = 8182,
    n_threads: int = 1,
    sim_duration: int = 1800 * 10,
    seed: int = 42,
    screen: int = 0,
):
    # print(f"Map Name: {map_filepath}")
    map_data, width, height = ArgosConfig.parse_map_file(map_filepath)

    # Transform the map and scen to Argos config file, obstacles: '@', 'T'
    if screen > 0:
        print("Creating Argos config file ...")
    robot_init_pos = init_start_locations(map_data, num_agents)

    ArgosConfig.create_Argos(
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
        screen=screen,
    )
    if screen > 0:
        print("Argos config file created.")

    try:
        print("Running simulator ...")
        server_executable_path = "./server/build/ADG_server"
        server_command = [
            server_executable_path,
            f"--num_robots={num_agents}",
            f"--port_number={port_num}",
            f"--output_file={stats_name}",
            f"--save_stats={str(save_stats).lower()}",
            f"--screen={screen}",
            f"--total_sim_step_tick={sim_duration}",
        ]
        client_command = ["argos3", "-c", f"../{argos_config_filepath}"]

        planner_executable_path = "./planner/PBS/build/pbs"
        planner_command = [
            planner_executable_path,
            f"--map={map_filepath}",
            f"--agentNum={num_agents}",
            f"--portNum={port_num}",
            f"--seed={seed}",
            f"--screen={screen}",
        ]
        run_simulator((server_command, client_command, planner_command))
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping the experiment ...")


if __name__ == "__main__":
    fire.Fire(main)
