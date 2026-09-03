export type Planner = "RHCR" | "PBS" | "TPBS";

export type TaskAssigner =
  | "windowed"
  | "distinct_one_goal"
  | "one_goal";

export type RunConfig = Readonly<{
  map_filepath: string;
  num_agents: number;
  n_threads: number;
  sim_duration: number;
  stop_at_congestion: boolean;
  sim_window_tick: number;
  ticks_per_second: number;
  velocity: number;
  planner: Planner;
  seed: number;
  screen: number;
  backup_solver: "PIBT" | "GuidedPIBT" | "LRAStar";
  planner_invoke_policy: "default" | "no_action";
  task_assigner_type: TaskAssigner;
  planning_window: number;
  cutoffTime: number;
  solver: "PBS" | "ECBS" | "WHCA" | "LRA" | "PIBT";
  single_agent_solver: "ASTAR" | "SIPP";
  rotation: boolean;
  rotation_time: number;
  prioritize_start: boolean;
  suboptimal_bound: number;
}>;

export type MapContents = {
  layout: string[];
  n_col: number;
  n_row: number;
  [key: string]: unknown;
};
