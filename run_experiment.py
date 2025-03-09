import os
import subprocess
import time
from multiprocessing import Process
import glob
import re
import multiprocessing
import time
def run_experiment(args):
    server_command, client_command = args
    # Start the server process
    server_process = subprocess.Popen(server_command)
    
    # Wait for a short period to ensure the server has started
    time.sleep(200)
    # Start the client process
    os.chdir("client")
    client_process = subprocess.Popen(client_command)
    
    # Wait for the client process to complete
    client_process.wait()
    os.chdir("..")
    server_process.wait()
    
    # # Terminate the server process
    # server_process.terminate()
# def run_experiment(server_command, client_command):
#     # Start the server process
#     server_process = subprocess.Popen(server_command)
    
#     # Wait for a short period to ensure the server has started
#     time.sleep(8)

#     # Start the client process
#     os.chdir("client")
#     client_process = subprocess.Popen(client_command)
    
#     # Wait for the client process to complete
#     client_process.wait()
#     os.chdir("..")
    
#     # # Terminate the server process
#     # server_process.terminate()
def extract_random_value(filename):
    match = re.search(r'random-(\d+)\.argos', filename)
    return int(match.group(1)) if match else float('inf')

def extract_scen_value(filename):
    match = re.search(r'scen_(\d+)\_paths.txt', filename)
    return int(match.group(1)) if match else float('inf')

def extract_method_name(filename):
    match = re.search(r'output/([^/]+)/', filename)
    return match.group(1) if match else "default"

def extract_map_name(filename):
    match = re.search(r'output/([^/]+)/([^/]+)/', filename)
    return match.group(2) if match else "default"
def extract_agent_num(dirname):
    match = re.search(r'(\d+)$', dirname)
    return int(match.group(1)) if match else float('inf')
def filter_out_already_run(agent_number, server_dir_list):
    index = agent_number//20
    return server_dir_list[index:]

if __name__ == "__main__":
    start_time = time.time()
    server_executable_path = "./server/build/ADG_server"
    client_executable_path = "./client"
    client_experiments_map_path = "./mapTransformer/output/Boston_0_256"
    server_experiments_map_path = "./output/PBS_astar/Boston_0_256/path"
    client_experiments_map_agent_list = sorted(glob.glob(os.path.join(client_experiments_map_path, "*")), key=extract_agent_num)
    server_experiments_map_agent_list = sorted(glob.glob(os.path.join(server_experiments_map_path, "*")), key=extract_agent_num)
    server_experiments_map_agent_list = filter_out_already_run(700, server_experiments_map_agent_list)
    for server_experiments_dir in server_experiments_map_agent_list:
        agent_value = extract_agent_num(server_experiments_dir)
        client_experiments_dir = client_experiments_map_agent_list[agent_value//20-1]
        client_configs = sorted(glob.glob(os.path.join(client_experiments_dir, "*.argos")), key=extract_random_value)
        server_list = sorted(glob.glob(os.path.join(server_experiments_dir, "*.txt")), key=extract_scen_value)
        pool = multiprocessing.Pool()
        port_number = 37000
        args_list = []
        num_experiment = min(len(server_list), len(client_configs))
        for server in server_list:
            scen_value = extract_scen_value(server)
            config = client_configs[scen_value-1]
            MAPF_method = extract_method_name(server)
            map_name = extract_map_name(server)
            if os.path.exists(f"server_output/{MAPF_method}/{map_name}"):
                os.mkdir(f"server_output/{MAPF_method}/{map_name}")
            server_output_path = f"server_output/{MAPF_method}/{map_name}/{scen_value}.csv"
            server_command = [server_executable_path, "-p", server, "-n", str(port_number+scen_value), "-o", server_output_path, "-m", map_name, "-s", str(scen_value), f"method_name={MAPF_method}"]
            client_command = ["argos3", "-c", f"../{config}"]
            args_list.append((server_command, client_command))
        pool.map(run_experiment, args_list)
        pool.close()
        pool.join()
    end_time = time.time()
    total_runtime = end_time - start_time
    print("Total runtime:", total_runtime, "seconds")
    # client_experiments_directory_path = "./mapTransformer/output/empty-32-32/80"
    # client_configs = sorted(glob.glob(os.path.join(client_experiments_directory_path, "*.argos")), key=extract_random_value)
    # server_paths_directory_path = "./output/PBS_astar/empty-32-32/path/80"
    # server_list = sorted(glob.glob(os.path.join(server_paths_directory_path, "*.txt")), key=extract_scen_value)

    # pool = multiprocessing.Pool()
    # port_number = 37000
    # args_list = []
    # num_experiment = min(len(server_list), len(client_configs))
    # for server in server_list:
    #     scen_value = extract_scen_value(server)
    #     config = client_configs[scen_value-1]
    #     MAPF_method = extract_method_name(server)
    #     map_name = extract_map_name(server)
    #     server_output_path = f"server_output/{MAPF_method}/{map_name}/{scen_value}.csv"
    #     server_command = [server_executable_path, "-p", server, "-n", str(port_number+scen_value), "-o", server_output_path, "-m", map_name, "-s", str(scen_value), f"method_name={MAPF_method}"]
    #     client_command = ["argos3", "-c", f"../{config}"]
    #     args_list.append((server_command, client_command))

    # pool.map(run_experiment, args_list)
    # pool.close()
    # pool.join()
    # end_time = time.time()
    # total_runtime = end_time - start_time
    # print("Total runtime:", total_runtime, "seconds")

# Existing code


# import os
# import subprocess
# import time
# from multiprocessing import Process
# import glob
# import re
# import time

# def run_experiment(server_command, client_command):
#     # Start the server process
#     server_process = subprocess.Popen(server_command)
    
#     # Wait for a short period to ensure the server has started
#     time.sleep(5)

#     # Start the client process
#     os.chdir("client")
#     client_process = subprocess.Popen(client_command)
    
#     # Wait for the client process to complete
#     client_process.wait()
#     os.chdir("..")
    
#     # Terminate the server process
#     server_process.terminate()
#     print("process end: ", server_command)

# def extract_random_value(filename):
#     match = re.search(r'random-(\d+)\.argos', filename)
#     return int(match.group(1)) if match else float('inf')

# def extract_scen_value(filename):
#     match = re.search(r'scen_(\d+)\_paths.txt', filename)
#     return int(match.group(1)) if match else float('inf')

# def extract_method_name(filename):
#     match = re.search(r'output/([^/]+)/', filename)
#     return match.group(1) if match else "default"

# def extract_map_name(filename):
#     match = re.search(r'output/([^/]+)/([^/]+)/', filename)
#     return match.group(2) if match else "default"
# if __name__ == "__main__":
#     start_time = time.time()
#     server_executable_path = "./server/build/ADG_server"
#     client_executable_path = "./client"

#     client_experiments_directory_path = "./mapTransformer/output/empty-32-32/20"

#     client_configs = sorted(glob.glob(os.path.join(client_experiments_directory_path, "*.argos")), key=extract_random_value)

#     server_paths_directory_path = "./output/PBS_astar/empty-32-32/path/20"
#     server_list = sorted(glob.glob(os.path.join(server_paths_directory_path, "*.txt")), key=extract_scen_value)
#     # client_output/PBS_astar_1_1/map/agent_number/scen_csv
#     # client_output/PBS_astar_1_2/map/agent_number/csv

#     # server_output/PBS_astar/map/csv
#     processes = []
#     port_number = 37001
#     for server, config in zip(server_list, client_configs):
#         MAPF_method = extract_method_name(server)
#         map_name = extract_map_name(server)
#         scen_value = extract_scen_value(server)
#         server_output_path = f"server_output/{MAPF_method}/{map_name}/{scen_value}.csv"
#         print(type(scen_value))
#         server_command = [server_executable_path, "-p", server, "-n", str(37000+scen_value), "-o", server_output_path, "-m", map_name, "-s", str(scen_value), f"method_name={MAPF_method}"]
#         port_number += 1
#         client_command = ["argos3", "-c", f"../{config}"]
#         # print(server_command)
#         # print(client_command)
        
#         # continue
#         process = Process(target=run_experiment, args=(server_command, client_command))
#         process.start()
#         processes.append(process)

#     for process in processes:
#         process.join()

#     end_time = time.time()
#     total_runtime = end_time - start_time
#     print("Total runtime:", total_runtime, "seconds")
