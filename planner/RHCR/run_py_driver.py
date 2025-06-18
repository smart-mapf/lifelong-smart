import sys

sys.path.append('build')
import fire
import json
import numpy as np
import warehouse_sim  # type: ignore # ignore pylance warning


def main(
    map_filepath,
    agent_file="",
    task_file="",
    agent_num=50,
    seed=0,
    simulation_time=1000,
    scenario="GREYORANGE",
):
    np.random.seed(seed)

    # Read in map
    with open(map_filepath, "r") as f:
        raw_env_json = json.load(f)
    map_json_str = json.dumps(raw_env_json)

    ret = warehouse_sim.run(
        map=map_json_str,
        agents=agent_file,
        task=task_file,
        output="output",
        scenario=scenario,
        agentNum=agent_num,
        cutoffTime=10,
        n_iter_limit=50,
        seed=0,
        screen=2,
        solver="WHCA",
        id=False,
        single_agent_solver="SIPP",
        lazyP=False,
        simulation_time=simulation_time,
        simulation_window=32,
        planning_window=48,
        travel_time_window=0,
        potential_function="NONE",
        potential_threshold=0,
        rotation=True,
        robust=0,
        CAT=False,
        hold_endpoints=False,
        dummy_paths=False,
        prioritize_start=True,
        suboptimal_bound=1,
        log=False,
        test=False,
        force_new_logdir=True,
        save_result=True,
        save_solver=False,
        save_heuristics_table=False,
        stop_at_traffic_jam=True,
        left_w_weight=1,
        right_w_weight=1,
        rotation_time=4,
        queue_mechanism=False,
    )

    analysis = json.loads(ret)
    print(analysis.keys())
    stop_at_t = analysis["stop_at_timestep"]
    print("stop at timestep", stop_at_t)

    vertex_wait_matrix = np.asarray(analysis["vertex_wait_matrix"])
    edge_usage_matrix = np.asarray(analysis["edge_usage_matrix"])
    cr_usage_matrix = np.asarray(analysis["cr_usage_matrix"])
    ccr_usage_matrix = np.asarray(analysis["ccr_usage_matrix"])

    # Check if the matrices are recording the stats properly
    h, w = raw_env_json["n_row"], raw_env_json["n_col"]
    vertex_wait_matrix = vertex_wait_matrix.reshape(h, w)
    edge_usage_matrix = edge_usage_matrix.reshape(h, w, 4)
    edge_usage_matrix = np.sum(edge_usage_matrix, axis=2)
    cr_usage_matrix = cr_usage_matrix.reshape(h, w)
    ccr_usage_matrix = ccr_usage_matrix.reshape(h, w)
    # The total number of actions should be equal to the number of agents * num
    # of timesteps
    if scenario == "KIVA":
        assert np.sum(vertex_wait_matrix + edge_usage_matrix +
                      cr_usage_matrix +
                      ccr_usage_matrix) == agent_num * stop_at_t

    n_wait_per_ag = np.sum(vertex_wait_matrix) / agent_num / stop_at_t
    n_lra_calls = analysis["n_rule_based_calls"]
    n_mapf_calls = analysis["n_mapf_calls"]

    print("throughput", analysis["throughput"])
    print("avg num rotations", analysis["num_rotations_mean"])
    print("% Agents wait per timestep", n_wait_per_ag)
    print(f"N LRA calls / N MAPF calls: {n_lra_calls} / {n_mapf_calls}")

    # if "finished_tasks" in analysis:
    #     finished_tasks = analysis["finished_tasks"]
    #     print("finished tasks", finished_tasks)

    if "path_inefficiency" in analysis:
        path_inefficiency = analysis["path_inefficiency"]
        print("path inefficiency", path_inefficiency)

if __name__ == "__main__":
    fire.Fire(main)
