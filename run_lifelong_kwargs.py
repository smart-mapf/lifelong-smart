import json
import fire

from run_lifelong import run_lifelong_argos


def run_lifelong_kwargs(kwargs_file):
    """
    Run lifelong MAPF with the given keyword arguments from a file.

    Args:
        kwargs_file (str): Path to the file containing keyword arguments.
    """
    with open(kwargs_file, 'r') as f:
        kwargs = json.load(f)

    file_name = "debug"

    map_filepath = file_name + "_map.json"
    stats_filepath = file_name + "_stats.json"
    argos_config_filepath = file_name + ".argos"

    # Write map file to a temporary file
    map_json_str = kwargs["map"]
    map_json = json.loads(map_json_str)
    with open(map_filepath, 'w') as f:
        json.dump(map_json, f)

    run_lifelong_argos(
        map_filepath=map_filepath,
        num_agents=kwargs["num_agents"],
        # headless=kwargs["headless"],
        headless=True,
        argos_config_filepath=argos_config_filepath,
        stats_name=stats_filepath,
        save_stats=kwargs["save_stats"],
        output_log=None,
        port_num=8282,
        n_threads=kwargs["n_threads"],
        sim_duration=kwargs["sim_duration"],
        sim_window_tick=kwargs["sim_window_tick"],
        planning_window=kwargs["planning_window"],
        velocity=kwargs["velocity"],
        # look_ahead_dist=kwargs["look_ahead_dist"],
        planner=kwargs["planner"],
        # container=kwargs["container"],
        container=False,
        seed=kwargs["seed"],
        # screen=kwargs["screen"],
        screen=1,
        # RHCR parameters
        # scenario=kwargs["scenario"],
        # task=kwargs["task"],
        cutoffTime=kwargs["cutoffTime"],
        solver=kwargs["solver"],
        backup_solver=kwargs["backup_solver"],
        # id=kwargs["id"],
        single_agent_solver=kwargs["single_agent_solver"],
        # lazyP=kwargs["lazyP"],
        # travel_time_window=kwargs["travel_time_window"],
        # potential_function=kwargs["potential_function"],
        # potential_threshold=kwargs["potential_threshold"],
        rotation=kwargs["rotation"],
        rotation_time=kwargs["rotation_time"],
        # robust=kwargs["robust"],
        # CAT=kwargs["CAT"],
        # hold_endpoints=kwargs["hold_endpoints"],
        # dummy_paths=kwargs["dummy_paths"],
        # prioritize_start=kwargs["prioritize_start"],
        suboptimal_bound=kwargs["suboptimal_bound"],
        log=kwargs["log"],
        save_result=kwargs["save_result"],
        save_solver=kwargs["save_solver"],
        save_heuristics_table=kwargs["save_heuristics_table"],
        # left_w_weight=kwargs["left_w_weight"],
        # right_w_weight=kwargs["right_w_weight"],
        # grid_type=kwargs["grid_type"],
        planner_invoke_policy=kwargs["planner_invoke_policy"],
        task_assigner_type=kwargs["task_assigner_type"],
        pibt_max_replan=kwargs["pibt_max_replan"],
        pibt_order_strategy=kwargs["pibt_order_strategy"],
        pibt_window_size=kwargs["pibt_window_size"],
        pibt_sim_window=kwargs["pibt_sim_window"],
        epibt_use_winpibt_revisit_inc=kwargs["epibt_use_winpibt_revisit_inc"],
        winpibt_max_bump_agents=kwargs["winpibt_max_bump_agents"],
        winpibt_get_path_mode=kwargs["winpibt_get_path_mode"],
        winpibt_allow_replan_l2h=kwargs["winpibt_allow_replan_l2h"],
        winpibt_allow_multi_hard_fail=kwargs["winpibt_allow_multi_hard_fail"],
        winpibt_add_soft_edges=kwargs["winpibt_add_soft_edges"],
        winpibt_SF_parent_selection=kwargs["winpibt_SF_parent_selection"],
        winpibt_HF_parent_selection=kwargs["winpibt_HF_parent_selection"],
    )


if __name__ == "__main__":
    fire.Fire(run_lifelong_kwargs)
