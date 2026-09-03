import { describe, expect, test } from "bun:test";
import { mkdtempSync, writeFileSync } from "fs";
import { tmpdir } from "os";
import { join } from "path";
import { loadMap, parseRunConfig } from "./config";

function config(args: string[] = []) {
  const result = parseRunConfig(args);
  if (result.help) throw new Error("unexpected help");
  return result.config;
}

describe("parseRunConfig", () => {
  test("uses defaults and planner-specific assigners", () => {
    expect(config()).toMatchObject({
      num_agents: 100,
      sim_duration: 6000,
      stop_at_congestion: true,
      task_assigner_type: "windowed",
    });
    expect(config(["--planner=PBS"]).task_assigner_type).toBe(
      "distinct_one_goal"
    );
    expect(config(["--planner=TPBS"]).task_assigner_type).toBe("one_goal");
  });

  test("normalizes backup solvers case-insensitively", () => {
    expect(config(["--backup_solver=pibt"]).backup_solver).toBe("PIBT");
    expect(config(["--backup_solver=guidedpibt"]).backup_solver).toBe(
      "GuidedPIBT"
    );
    expect(config(["--backup_solver=lrastar"]).backup_solver).toBe("LRAStar");
  });

  test("parses values, booleans, and aliases", () => {
    const parsed = config([
      "--map_filepath=/workspace/custom.json",
      "--num_agents=12",
      "--n_threads=4",
      "--sim_duration=1200",
      "--stop_at_congestion=false",
      "--sim_window_tick=30",
      "--ticks_per_second=10",
      "--velocity=150.5",
      "--planner=RHCR",
      "--seed=99",
      "--screen=2",
      "--backup_solver=LRA",
      "--planner_invoke_policy=no_action",
      "--task_assigner_type=windowed",
      "--planning_window=25",
      "--cutoffTime=3",
      "--solver=ecbs",
      "--single_agent_solver=astar",
      "--rotation=true",
      "--rotation_time=2",
      "--prioritize_start=false",
      "--suboptimal_bound=1.5",
    ]);
    expect(parsed).toEqual({
      map_filepath: "/workspace/custom.json",
      num_agents: 12,
      n_threads: 4,
      sim_duration: 1200,
      stop_at_congestion: false,
      sim_window_tick: 30,
      ticks_per_second: 10,
      velocity: 150.5,
      planner: "RHCR",
      seed: 99,
      screen: 2,
      backup_solver: "LRAStar",
      planner_invoke_policy: "no_action",
      task_assigner_type: "windowed",
      planning_window: 25,
      cutoffTime: 3,
      solver: "ECBS",
      single_agent_solver: "ASTAR",
      rotation: true,
      rotation_time: 2,
      prioritize_start: false,
      suboptimal_bound: 1.5,
    });

    expect(
      config([
        "--planner=PBS",
        "--task_assigner_type=distinct-one-goal",
      ]).task_assigner_type
    ).toBe("distinct_one_goal");
    expect(
      config(["--planner=TPBS", "--task_assigner_type=one-goal"])
        .task_assigner_type
    ).toBe("one_goal");
  });

  test("rejects unsupported and incompatible options", () => {
    expect(() => config(["--planner=MASS"])).toThrow("CPLEX");
    expect(() =>
      config(["--planner=PBS", "--task_assigner_type=windowed"])
    ).toThrow("PBS requires");
    expect(() => config(["--num_agents=0"])).toThrow("num_agents");
    expect(() => config(["--n_threads=0"])).toThrow("n_threads");
    expect(() => config(["--rotation=yes"])).toThrow("true or false");
    expect(() => config(["--stop_at_congestion=no"])).toThrow(
      "true or false"
    );
    expect(() => config(["--unknown=value"])).toThrow();
    expect(() =>
      config([
        "--sim_window_tick=5",
        "--ticks_per_second=10",
        "--cutoffTime=1",
      ])
    ).toThrow("cutoffTime");
  });

  test("recognizes help", () => {
    expect(parseRunConfig(["--help"])).toEqual({ help: true });
  });
});

describe("loadMap", () => {
  test("loads a valid map and rejects malformed maps", () => {
    const root = mkdtempSync(join(tmpdir(), "lsmart-config-"));
    writeFileSync(
      join(root, "valid.json"),
      JSON.stringify({ n_row: 1, n_col: 2, layout: [".."] })
    );
    writeFileSync(
      join(root, "invalid.json"),
      JSON.stringify({ n_row: 2, n_col: 2, layout: [".."] })
    );

    expect(loadMap("valid.json", root).mapContents.layout).toEqual([".."]);
    expect(() => loadMap("invalid.json", root)).toThrow("Invalid LSMART map");
    expect(() => loadMap("missing.json", root)).toThrow("not found");
  });
});
