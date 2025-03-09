import os
import pathlib
import argparse
import ArgosConfig
import subprocess
import time
import psutil
import csv
import json
from datetime import datetime
import gc
from logdir import LogDir
from multiprocessing import Process


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Argument parser for map_name and scen_name.")
    parser.add_argument("--map_name",
                        type=str,
                        required=True,
                        help="Name of the map file")
    parser.add_argument("--scen_name",
                        type=str,
                        required=True,
                        help="Name of the scenario file")
    parser.add_argument("--num_agents",
                        type=int,
                        required=True,
                        help="Number of agents in the scenario")
    parser.add_argument("--headless",
                        type=bool,
                        required=False,
                        default=False,
                        help="Simulator run in headless mode")
    parser.add_argument("--argos_config_name",
                        type=str,
                        required=False,
                        default="output.argos",
                        help="Name of the argos config file")
    parser.add_argument("--path_filename",
                        type=str,
                        required=False,
                        default="outputPath.txt",
                        help="Name of the output path file")
    parser.add_argument("--stats_name",
                        type=str,
                        required=False,
                        default="stats.csv",
                        help="Name of the statistics file for simulator")
    parser.add_argument("--port_num",
                        type=int,
                        required=False,
                        default=8182,
                        help="Port number for sim and client")
    parser.add_argument("--n_threads",
                        type=int,
                        required=False,
                        default=0,
                        help="Number of threads for argos")
    parser.add_argument("--monitor_interval",
                        type=int,
                        required=False,
                        default=5,
                        help="Interval for monitoring system")

    return parser.parse_args()


def monitor_system(save_dir: pathlib.Path, interval: int = 5):
    """
    A function that logs CPU and RAM usage every `interval` seconds.
    """
    # Create a log file
    sys_log_file = save_dir / "sys_monitor.csv"

    baseline_cpu = psutil.cpu_percent(interval=None)  # system-wide CPU usage %
    baseline_ram = psutil.virtual_memory().used  # used RAM in bytes
    print(
        f"Baseline CPU: {baseline_cpu}% | RAM: {baseline_ram / (1024 ** 3)} GB"
    )

    with open(sys_log_file, mode="a", newline="") as f:
        writer = csv.writer(f)

        # Write the header
        writer.writerow(["timestamp", "cpu_usage_percent", "ram_usage"])

        # time.sleep(interval)  # Sleep over the first interval

        while True:
            # Current CPU usage % (system-wide) and used RAM in bytes
            current_cpu = psutil.cpu_percent(interval=None)
            current_ram = psutil.virtual_memory().used

            # Compute the difference
            diff_cpu_percent = current_cpu
            diff_ram_gb = (current_ram - baseline_ram) / (1024**3)

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # Write a single row to the CSV file
            writer.writerow([timestamp, diff_cpu_percent, diff_ram_gb])

            # Flush the file buffer so data is written immediately
            f.flush()

            # print(f"[Monitor] CPU: {cpu_usage}% | RAM: {ram_usage}%")
            time.sleep(interval)


def run_planner(command: list):
    # Combine executable path and arguments into a single list
    # print(command)
    try:
        # Run the command and capture the output
        result = subprocess.run(command,
                                capture_output=True,
                                text=True,
                                check=True)

        # Print any errors from the executable
        if result.stderr:
            print("Errors:")
            print(result.stderr)
            return 0

        if "Succeed" in result.stdout:
            return 1
        else:
            if "Fail" in result.stdout or "Timeout" in result.stdout:
                return 0
            else:
                return 1

    except subprocess.CalledProcessError as e:
        print(f"Execution failed: {e}")
        print(f"Error output: {e.stderr}")
        return 0


def check_file(file_path: str):
    if not os.path.exists(file_path):
        print(f"{file_path} not exists!")
        return False
    return True


def run_simulator(args):
    server_command, client_command = args
    # Start the server process
    server_process = subprocess.Popen(server_command)

    # Wait for a short period to ensure the server has started
    time.sleep(1)
    # Start the client process
    os.chdir("MAPF-Benchmark-Argos/client")
    # os.chdir("client")

    client_process = subprocess.Popen(client_command)

    # Wait for the client process to complete
    client_process.wait()
    os.chdir("..")
    server_process.wait()


def main():
    args = parse_arguments()
    print(f"Map Name: {args.map_name}")
    print(f"Scenario Name: {args.scen_name}")

    # Configs
    scen_file_path = args.scen_name
    map_file_path = args.map_name
    config_filename = args.argos_config_name
    curr_num_agent = args.num_agents
    port_num = args.port_num
    path_filename = args.path_filename
    sim_stats_filename = args.stats_name
    n_threads = args.n_threads
    monitor_interval = args.monitor_interval

    if not check_file(map_file_path):
        print("Map file not exists!")
        exit(-1)
    if not check_file(scen_file_path):
        print("Scenario file not exists!")
        exit(-1)

    # -1 will use all available threads
    if n_threads == -1:
        # minus 1 for the main thread
        n_threads = psutil.cpu_count(logical=True) - 1

    # Create logdir
    map_name = os.path.splitext(os.path.basename(map_file_path))[0]
    logdir = LogDir(f"run_sim_{map_name}_{curr_num_agent}",
                    rootdir="./logs",
                    uuid=8)

    # Save configuration options.
    with logdir.pfile("config.json").open("w") as file:
        # Save args as json
        json.dump(vars(args), file, indent=4)

    # Manipulate the filepaths
    config_filename = logdir.logdir / config_filename
    config_filename = config_filename.resolve().as_posix()
    path_filename = logdir.logdir / path_filename
    path_filename = path_filename.resolve().as_posix()
    sim_stats_filename = logdir.logdir / sim_stats_filename
    sim_stats_filename = sim_stats_filename.resolve().as_posix()

    # Transform the map and scen to Argos config file, obstacles: '@', 'T'
    print("Creating Argos config file ...")
    robot_init_pos, scen_num_agent = ArgosConfig.read_scen(scen_file_path)
    map_data, width, height = ArgosConfig.parse_map_file(map_file_path)
    if scen_num_agent < curr_num_agent:
        print("Number of agents exceed maximum number. exiting ...")
        exit(-1)
    ArgosConfig.create_Argos(map_data, config_filename, width, height,
                             robot_init_pos, curr_num_agent, port_num,
                             n_threads, not args.headless)
    print("Argos config file created.")

    # Run planner to find solution for the specified problem
    print("Running planner ...")
    executable_path = "/usr/project/MAPF-Benchmark-Argos/planner/LNS/build/lns"
    run_args = []
    run_args += [executable_path]
    run_args += ["-m", map_file_path]
    run_args += ["-a", scen_file_path]
    run_args += ["--outputPaths=" + path_filename]
    run_args += ["-k", str(curr_num_agent)]
    run_args += ["-t", "300"]
    run_args += [f"--maxIterations=1000"]
    run_planner(run_args)
    print("Planner create path successful.")
    gc.collect()
    time.sleep(5)  # Wait for gc to release memory

    # Start the monitor process
    monitoring_process = Process(target=monitor_system,
                                 args=(logdir.logdir, monitor_interval))
    monitoring_process.start()

    try:
        print("Running simulator ...")
        server_executable_path = "/usr/project/MAPF-Benchmark-Argos/server/build/ADG_server"
        # server_executable_path = "server/build/ADG_server"
        server_command = [
            server_executable_path, "-p", path_filename, "-n",
            str(port_num), "-o", sim_stats_filename, "-m", map_file_path, "-s",
            str(scen_file_path), f"--method_name=LNS2"
        ]
        client_command = ["argos3", "-c", f"{config_filename}"]
        print(client_command)
        run_simulator((server_command, client_command))
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping the experiment ...")
    finally:
        if monitoring_process.is_alive():
            print("[Main] Stopping the monitoring process...")
            monitoring_process.terminate()
            monitoring_process.join()
            print("[Main] Monitoring process stopped.")


if __name__ == "__main__":
    main()
