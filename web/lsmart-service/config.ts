import { existsSync, readFileSync } from "fs";
import { isAbsolute, resolve } from "path";
import { parseArgs } from "util";
import type {
  MapContents,
  Planner,
  RunConfig,
  TaskAssigner,
} from "./types";

export type {
  MapContents,
  Planner,
  RunConfig,
  TaskAssigner,
} from "./types";

export const HELP = `Usage: lsmart-viz [options]

Simulation:
  --map_filepath PATH          Bundled or mounted LSMART map
  --num_agents N               Number of robots (default: 100)
  --n_threads N                Total threads; 1 disables ARGoS threading (default: 1)
  --sim_duration N             Simulation ticks (default: 6000)
  --stop_at_congestion BOOL    Stop when congestion is detected (default: true)
  --sim_window_tick N          Planner invocation window in ticks (default: 20)
  --ticks_per_second N         Simulation tick rate (default: 10)
  --velocity N                 Robot velocity in cm/s (default: 200)
  --planner NAME               RHCR, PBS, or TPBS (default: RHCR)
  --seed N                     Random seed (default: 42)
  --screen N                   Logging level: 0, 1, or 2 (default: 0)
  --backup_solver NAME         PIBT, GuidedPIBT, or LRAStar (default: PIBT)
  --planner_invoke_policy NAME default or no_action (default: default)
  --task_assigner_type NAME    windowed, distinct_one_goal, or one_goal
  --planning_window N          Planning window in timesteps (default: 10)
  --cutoffTime N               Planner cutoff in seconds (default: 1)

RHCR:
  --solver NAME                PBS, ECBS, WHCA, LRA, or PIBT (default: PBS)
  --single_agent_solver NAME   ASTAR or SIPP (default: SIPP)
  --rotation BOOL              true or false (default: false)
  --rotation_time N            Rotation timesteps (default: 1)
  --prioritize_start BOOL      true or false (default: true)
  --suboptimal_bound N         ECBS bound, at least 1 (default: 1)

  --help                       Show this help
`;

const STRING_OPTIONS = {
  map_filepath: { type: "string" },
  num_agents: { type: "string" },
  n_threads: { type: "string" },
  sim_duration: { type: "string" },
  stop_at_congestion: { type: "string" },
  sim_window_tick: { type: "string" },
  ticks_per_second: { type: "string" },
  velocity: { type: "string" },
  planner: { type: "string" },
  seed: { type: "string" },
  screen: { type: "string" },
  backup_solver: { type: "string" },
  planner_invoke_policy: { type: "string" },
  task_assigner_type: { type: "string" },
  planning_window: { type: "string" },
  cutoffTime: { type: "string" },
  solver: { type: "string" },
  single_agent_solver: { type: "string" },
  rotation: { type: "string" },
  rotation_time: { type: "string" },
  prioritize_start: { type: "string" },
  suboptimal_bound: { type: "string" },
  help: { type: "boolean" },
} as const;

function numberValue(
  name: string,
  value: string | undefined,
  fallback: number,
  options: { integer?: boolean; min?: number; max?: number } = {}
): number {
  if (value === undefined) return fallback;
  const parsed = Number(value);
  if (
    !Number.isFinite(parsed) ||
    (options.integer && !Number.isInteger(parsed)) ||
    (options.min !== undefined && parsed < options.min) ||
    (options.max !== undefined && parsed > options.max)
  ) {
    const range =
      options.min !== undefined || options.max !== undefined
        ? ` in range ${options.min ?? "-∞"}..${options.max ?? "∞"}`
        : "";
    throw new Error(
      `--${name} must be ${options.integer ? "an integer" : "a number"}${range}`
    );
  }
  return parsed;
}

function booleanValue(
  name: string,
  value: string | undefined,
  fallback: boolean
): boolean {
  if (value === undefined) return fallback;
  if (value === "true") return true;
  if (value === "false") return false;
  throw new Error(`--${name} must be true or false`);
}

function enumValue<T extends string>(
  name: string,
  value: string | undefined,
  fallback: T,
  allowed: readonly T[]
): T {
  if (value === undefined) return fallback;
  if (allowed.includes(value as T)) return value as T;
  throw new Error(`--${name} must be one of: ${allowed.join(", ")}`);
}

function normalizeAssigner(value: string): string {
  return value.replaceAll("-", "_");
}

export function parseRunConfig(
  args: string[]
): { help: true } | { help: false; config: RunConfig } {
  const { values } = (() => {
    try {
      return parseArgs({
        args,
        options: STRING_OPTIONS,
        strict: true,
        allowPositionals: false,
      });
    } catch (error) {
      throw new Error(error instanceof Error ? error.message : String(error));
    }
  })();

  if (values.help) return { help: true };

  const plannerInput = (values.planner ?? "RHCR").toUpperCase();
  if (plannerInput === "MASS") {
    throw new Error(
      "MASS is unavailable in this image because CPLEX is not installed"
    );
  }
  const planner = enumValue(
    "planner",
    plannerInput,
    "RHCR",
    ["RHCR", "PBS", "TPBS"] as const
  );

  const expectedAssigner: Record<Planner, TaskAssigner> = {
    RHCR: "windowed",
    PBS: "distinct_one_goal",
    TPBS: "one_goal",
  };
  const assignerInput =
    values.task_assigner_type === undefined
      ? expectedAssigner[planner]
      : normalizeAssigner(values.task_assigner_type.toLowerCase());
  const task_assigner_type = enumValue(
    "task_assigner_type",
    assignerInput,
    expectedAssigner[planner],
    ["windowed", "distinct_one_goal", "one_goal"] as const
  );
  if (task_assigner_type !== expectedAssigner[planner]) {
    throw new Error(
      `${planner} requires --task_assigner_type=${expectedAssigner[planner]}`
    );
  }

  const backupInput = values.backup_solver ?? "PIBT";
  const backupAliases = {
    PIBT: "PIBT",
    GUIDEDPIBT: "GuidedPIBT",
    LRA: "LRAStar",
    LRASTAR: "LRAStar",
  } as const;
  const backupAlias =
    backupAliases[
      backupInput.toUpperCase() as keyof typeof backupAliases
    ] ?? backupInput;
  const config: RunConfig = Object.freeze({
    map_filepath: values.map_filepath ?? "maps/kiva_large_w_mode.json",
    num_agents: numberValue("num_agents", values.num_agents, 100, {
      integer: true,
      min: 1,
    }),
    n_threads: numberValue("n_threads", values.n_threads, 1, {
      integer: true,
      min: 1,
    }),
    sim_duration: numberValue("sim_duration", values.sim_duration, 6000, {
      integer: true,
      min: 1,
    }),
    stop_at_congestion: booleanValue(
      "stop_at_congestion",
      values.stop_at_congestion,
      true
    ),
    sim_window_tick: numberValue(
      "sim_window_tick",
      values.sim_window_tick,
      20,
      { integer: true, min: 1 }
    ),
    ticks_per_second: numberValue(
      "ticks_per_second",
      values.ticks_per_second,
      10,
      { integer: true, min: 1 }
    ),
    velocity: numberValue("velocity", values.velocity, 200, { min: 0.001 }),
    planner,
    seed: numberValue("seed", values.seed, 42, {
      integer: true,
      min: 0,
      max: 4294967295,
    }),
    screen: numberValue("screen", values.screen, 0, {
      integer: true,
      min: 0,
      max: 2,
    }),
    backup_solver: enumValue(
      "backup_solver",
      backupAlias,
      "PIBT",
      ["PIBT", "GuidedPIBT", "LRAStar"] as const
    ),
    planner_invoke_policy: enumValue(
      "planner_invoke_policy",
      values.planner_invoke_policy,
      "default",
      ["default", "no_action"] as const
    ),
    task_assigner_type,
    planning_window: numberValue(
      "planning_window",
      values.planning_window,
      10,
      { integer: true, min: 1 }
    ),
    cutoffTime: numberValue("cutoffTime", values.cutoffTime, 1, {
      integer: true,
      min: 1,
    }),
    solver: enumValue(
      "solver",
      values.solver?.toUpperCase(),
      "PBS",
      ["PBS", "ECBS", "WHCA", "LRA", "PIBT"] as const
    ),
    single_agent_solver: enumValue(
      "single_agent_solver",
      values.single_agent_solver?.toUpperCase(),
      "SIPP",
      ["ASTAR", "SIPP"] as const
    ),
    rotation: booleanValue("rotation", values.rotation, false),
    rotation_time: numberValue(
      "rotation_time",
      values.rotation_time,
      1,
      { integer: true, min: 1 }
    ),
    prioritize_start: booleanValue(
      "prioritize_start",
      values.prioritize_start,
      true
    ),
    suboptimal_bound: numberValue(
      "suboptimal_bound",
      values.suboptimal_bound,
      1,
      { min: 1 }
    ),
  });

  if (
    config.planner_invoke_policy === "default" &&
    config.cutoffTime > config.sim_window_tick / config.ticks_per_second
  ) {
    throw new Error(
      "--cutoffTime must not exceed sim_window_tick / ticks_per_second with the default invocation policy"
    );
  }

  return { help: false, config };
}

export function loadMap(
  mapFilepath: string,
  projectRoot: string
): { mapPath: string; mapContents: MapContents } {
  const mapPath = isAbsolute(mapFilepath)
    ? mapFilepath
    : resolve(projectRoot, mapFilepath);
  if (!existsSync(mapPath)) {
    throw new Error(`Map file not found: ${mapFilepath}`);
  }

  let parsed: unknown;
  try {
    parsed = JSON.parse(readFileSync(mapPath, "utf8"));
  } catch (error) {
    throw new Error(
      `Cannot parse map ${mapFilepath}: ${
        error instanceof Error ? error.message : String(error)
      }`
    );
  }

  if (
    typeof parsed !== "object" ||
    parsed === null ||
    !Array.isArray((parsed as MapContents).layout) ||
    !(parsed as MapContents).layout.every((row) => typeof row === "string") ||
    !Number.isInteger((parsed as MapContents).n_row) ||
    !Number.isInteger((parsed as MapContents).n_col) ||
    (parsed as MapContents).n_row <= 0 ||
    (parsed as MapContents).n_col <= 0 ||
    (parsed as MapContents).layout.length !== (parsed as MapContents).n_row ||
    !(parsed as MapContents).layout.every(
      (row) => row.length === (parsed as MapContents).n_col
    )
  ) {
    throw new Error(
      `Invalid LSMART map ${mapFilepath}: layout must match n_row and n_col`
    );
  }

  return { mapPath, mapContents: parsed as MapContents };
}
