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
    return [(str(start // w), str(start % w)) for start in starts]


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

    client_process.wait()
    server_process.wait()


def main(
    map_filepath: str = "../maps/mapf_bench/no_guidance/random-32-32-20.json",
    num_agents: int = 32,
    headless: bool = False,
    argos_config_filepath: str = "output.argos",
    stats_name: str = "stats.csv",
    port_num: int = 8182,
    n_threads: int = 1,
    sim_duration: int = 3600 * 10,
):
    print(f"Map Name: {map_filepath}")
    map_data, width, height = ArgosConfig.parse_map_file(map_filepath)

    # Transform the map and scen to Argos config file, obstacles: '@', 'T'
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
    )
    print("Argos config file created.")

    try:
        print("Running simulator ...")
        server_executable_path = "./server/build/ADG_server"
        server_command = [
            server_executable_path,
            f"--num_robots={num_agents}",
            f"--port_number={port_num}",
            f"--output_file={stats_name}",
        ]
        client_command = ["argos3", "-c", f"../{argos_config_filepath}"]
        print(client_command)

        # executable_path = "./planner/MAPF-LNS2/build/lns"
        executable_path = "./planner/PBS/build/pbs"
        run_args = []
        run_args += [executable_path]
        run_args += ["-m", f"./{map_filepath}"]
        run_args += ["-k", str(num_agents)]
        run_args += ["--portNum", str(port_num)]
        run_simulator((server_command, client_command, run_args))
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping the experiment ...")


if __name__ == "__main__":
    fire.Fire(main)
