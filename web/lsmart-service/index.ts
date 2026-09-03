#!/usr/bin/env bun

import { spawn, type ReadableSubprocess } from "bun";
import { load } from "js-yaml";
import { existsSync } from "fs";
import { createServer } from "net";
import { join, resolve } from "path";
import type { ServerWebSocket } from "bun";
import {
  HELP,
  loadMap,
  parseRunConfig,
  type MapContents,
  type RunConfig,
} from "./config";

// ─── Types ──────────────────────────────────────────────────────────────────

type Step = {
  type: "tick";
  clock: number;
  agents: {
    id: number;
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  }[];
};

type ExecProgress = {
  type: "exec_progress";
  agent: number;
  finished: number;
  total: number;
};

type StateChange = {
  type: "state_change";
  agent: number;
  value: "initialized" | "active" | "idle" | "finished";
};

type Stats = {
  type: "stats";
  [key: string]: unknown;
};

type MetaEvent = {
  type: "meta";
  map_contents: MapContents;
  map_format: "lsmart-json";
  map_path: string;
  agent_count: number;
  ticks_per_second: number;
  sim_duration: number;
  planner: string;
  task_assigner_type: string;
  backup_solver: string;
  effective_config: RunConfig;
};

type Output =
  | Step
  | ExecProgress
  | StateChange
  | Stats
  | MetaEvent
  | { type: "message"; content: string }
  | { type: "error"; error: string };

type ActiveRun = {
  proc: ReadableSubprocess;
  cancelled: boolean;
  simulatorPort: number;
};

const REPO_ROOT = resolve(import.meta.dir, "../..");

type SocketState =
  | { status: "ready" }
  | { status: "starting" }
  | { status: "running"; run: ActiveRun }
  | { status: "finished" };

const socketStates = new WeakMap<ServerWebSocket<unknown>, SocketState>();

// ─── Helpers ────────────────────────────────────────────────────────────────

async function* streamLines(
  stream: ReadableStream<Uint8Array>
): AsyncGenerator<string> {
  let leftover = "";
  const reader = stream.getReader();
  const decoder = new TextDecoder();

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    const text = leftover + decoder.decode(value);
    const lines = text.split(/\r?\n/);
    leftover = lines.pop()!;
    for (const line of lines) yield line;
  }
  if (leftover) yield leftover;
}

function isOutput(a: unknown): a is Output {
  return typeof a === "object" && a !== null && "type" in a;
}

async function getAvailablePort(): Promise<number> {
  return await new Promise((resolvePort, rejectPort) => {
    const server = createServer();

    server.once("error", rejectPort);
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      if (!address || typeof address === "string") {
        server.close(() => {
          rejectPort(new Error("Failed to allocate a simulator RPC port"));
        });
        return;
      }

      const { port } = address;
      server.close((err) => {
        if (err) {
          rejectPort(err);
          return;
        }
        resolvePort(port);
      });
    });
  });
}

function sendOutputs(ws: ServerWebSocket<unknown>, outputs: Output[]) {
  if (outputs.length === 0 || ws.readyState !== WebSocket.OPEN) {
    return;
  }

  ws.send(JSON.stringify(outputs.length === 1 ? outputs[0] : outputs));
}

function parseOutputLine(
  line: string,
  agentStates: Map<number, string>
): Output[] {
  if (!line.trim()) {
    return [];
  }

  const parseStructuredOutput = (parsed: unknown): Output[] => {
    if (!isOutput(parsed)) {
      return [{ type: "message", content: line }];
    }

    const outputBatch: Output[] = [];

    if (parsed.type === "exec_progress") {
      const agentId = parsed.agent;
      const prevState = agentStates.get(agentId);
      const isDone = parsed.finished >= parsed.total;

      if (isDone && prevState !== "idle") {
        agentStates.set(agentId, "idle");
        outputBatch.push({
          type: "state_change",
          agent: agentId,
          value: "idle",
        });
      } else if (!isDone && prevState !== "active") {
        agentStates.set(agentId, "active");
        outputBatch.push({
          type: "state_change",
          agent: agentId,
          value: "active",
        });
      }
    }

    if (parsed.type === "state_change" && parsed.value === "initialized") {
      agentStates.set(parsed.agent, "initialized");
    }

    outputBatch.push(parsed);
    return outputBatch;
  };

  try {
    return parseStructuredOutput(JSON.parse(line));
  } catch {
    try {
      return parseStructuredOutput(load(line));
    } catch {
      return [{ type: "message", content: line }];
    }
  }
}

// ─── Run Simulation ─────────────────────────────────────────────────────────

async function runSimulation(
  ws: ServerWebSocket<unknown>,
  config: RunConfig,
  mapPath: string
) {
  let simulatorPort: number;
  try {
    simulatorPort = await getAvailablePort();
  } catch (err) {
    sendOutputs(ws, [
      {
        type: "error",
        error: `Failed to allocate a simulator RPC port: ${String(err)}`,
      },
    ]);
    socketStates.set(ws, { status: "finished" });
    return;
  }
  sendOutputs(ws, [
    {
      type: "message",
      content: `[lsmart-service] Using simulator RPC port ${simulatorPort}`,
    },
  ]);

  if (ws.readyState !== WebSocket.OPEN) {
    socketStates.set(ws, { status: "finished" });
    return;
  }

  // Build command
  const extVizPluginDir = join(
    REPO_ROOT,
    "plugins/visualizers/external_visualizer/build"
  );
  const argosConfigPath = join("/tmp", `lsmart-${simulatorPort}.argos`);
  const args = [
    "run_lifelong.py",
    `--map_filepath=${mapPath}`,
    `--argos_config_filepath=${argosConfigPath}`,
    `--num_agents=${config.num_agents}`,
    `--n_threads=${config.n_threads}`,
    `--planner=${config.planner}`,
    `--planner_invoke_policy=${config.planner_invoke_policy}`,
    `--task_assigner_type=${config.task_assigner_type}`,
    `--backup_solver=${config.backup_solver}`,
    `--sim_duration=${config.sim_duration}`,
    `--stop_at_congestion=${config.stop_at_congestion}`,
    `--sim_window_tick=${config.sim_window_tick}`,
    `--ticks_per_second=${config.ticks_per_second}`,
    `--velocity=${config.velocity}`,
    `--seed=${config.seed}`,
    `--screen=${config.screen}`,
    `--planning_window=${config.planning_window}`,
    `--cutoffTime=${config.cutoffTime}`,
    `--rotation=${config.rotation}`,
    `--port_num=${simulatorPort}`,
    `--container=True`,
    `--external_visualization=True`,
    `--headless=False`,
    `--save_stats=False`,
  ];
  if (config.planner === "RHCR") {
    args.push(
      `--solver=${config.solver}`,
      `--single_agent_solver=${config.single_agent_solver}`,
      `--rotation_time=${config.rotation_time}`,
      `--prioritize_start=${config.prioritize_start}`,
      `--suboptimal_bound=${config.suboptimal_bound}`
    );
  }

  let proc: ReadableSubprocess;
  try {
    proc = spawn({
      cmd: ["python3", ...args],
      cwd: REPO_ROOT,
      stdout: "pipe",
      stderr: "pipe",
      env: {
        ...process.env,
        ARGOS_PLUGIN_PATH: extVizPluginDir,
      },
    });
  } catch (err) {
    sendOutputs(ws, [
      {
        type: "error",
        error: `Failed to start simulation: ${String(err)}`,
      },
    ]);
    socketStates.set(ws, { status: "finished" });
    return;
  }

  const runState: ActiveRun = {
    proc,
    cancelled: false,
    simulatorPort,
  };
  socketStates.set(ws, { status: "running", run: runState });

  if (ws.readyState !== WebSocket.OPEN) {
    runState.cancelled = true;
    try {
      proc.kill();
    } catch {}
    socketStates.set(ws, { status: "finished" });
    return;
  }

  // Track agent states for synthesizing active/idle
  const agentStates: Map<number, string> = new Map();

  const stdoutTask = (async () => {
    for await (const line of streamLines(proc.stdout)) {
      sendOutputs(ws, parseOutputLine(line, agentStates));
    }
  })();

  const stderrTask = (async () => {
    for await (const line of streamLines(proc.stderr)) {
      if (!line.trim()) {
        continue;
      }
      sendOutputs(ws, [{ type: "message", content: line.trim() }]);
    }
  })();

  let exited = false;
  try {
    const exitCode = await proc.exited;
    exited = true;
    await Promise.all([stdoutTask, stderrTask]);

    if (!runState.cancelled && exitCode !== 0) {
      sendOutputs(ws, [
        {
          type: "error",
          error: `Simulation exited with code ${exitCode} on port ${simulatorPort}`,
        },
      ]);
    }
  } catch (err) {
    sendOutputs(ws, [{ type: "error", error: String(err) }]);
  } finally {
    socketStates.set(ws, { status: "finished" });
    try {
      if (!exited) {
        proc.kill();
      }
    } catch {}
  }
}

// ─── HTTP + WebSocket Server ────────────────────────────────────────────────

export function startServer(config: RunConfig) {
  const { mapPath, mapContents } = loadMap(config.map_filepath, REPO_ROOT);
  const meta: MetaEvent = {
    type: "meta",
    map_contents: mapContents,
    map_format: "lsmart-json",
    map_path: config.map_filepath,
    agent_count: config.num_agents,
    ticks_per_second: config.ticks_per_second,
    sim_duration: config.sim_duration,
    planner: config.planner,
    task_assigner_type: config.task_assigner_type,
    backup_solver: config.backup_solver,
    effective_config: config,
  };

  const port = Number(process.env.PORT || "3000");
  if (!Number.isInteger(port) || port < 1 || port > 65535) {
    throw new Error("PORT must be an integer in range 1..65535");
  }
  const frontendDevServerUrl = process.env.LSMART_VISUALIZER_DEV_URL?.replace(
    /\/+$/,
    ""
  );
  const staticDir = join(import.meta.dir, "../lsmart-visualiser/dist");

  const server = Bun.serve({
    port,
    async fetch(req, server) {
      const url = new URL(req.url);

      if (url.pathname === "/ws") {
        if (server.upgrade(req)) return;
        return new Response("WebSocket upgrade required", { status: 426 });
      }

      if (frontendDevServerUrl) {
        return Response.redirect(
          `${frontendDevServerUrl}${url.pathname}${url.search}`,
          307
        );
      }

      let path = url.pathname;
      if (path === "/") path = "/index.html";
      const filePath = join(staticDir, path);
      if (existsSync(filePath)) return new Response(Bun.file(filePath));
      return new Response("Not Found", { status: 404 });
    },
    websocket: {
      open(ws) {
        console.log("[lsmart-service] Client connected, waiting to start...");
        socketStates.set(ws, { status: "ready" });
        ws.send(JSON.stringify(meta));
      },
      close(ws) {
        console.log("[lsmart-service] Client disconnected");
        const state = socketStates.get(ws);
        if (state?.status === "running") {
          state.run.cancelled = true;
          try {
            state.run.proc.kill();
          } catch {}
        }
        socketStates.delete(ws);
      },
      message(ws, message) {
        let input: unknown;
        try {
          const text =
            typeof message === "string"
              ? message
              : new TextDecoder().decode(message);
          input = JSON.parse(text);
        } catch {
          sendOutputs(ws, [
            { type: "error", error: "Expected JSON message {\"type\":\"start\"}" },
          ]);
          return;
        }

        if (
          typeof input !== "object" ||
          input === null ||
          (input as { type?: unknown }).type !== "start" ||
          Object.keys(input).length !== 1
        ) {
          sendOutputs(ws, [
            { type: "error", error: "Expected message {\"type\":\"start\"}" },
          ]);
          return;
        }

        if (socketStates.get(ws)?.status !== "ready") {
          sendOutputs(ws, [
            { type: "error", error: "Simulation has already been started" },
          ]);
          return;
        }

        socketStates.set(ws, { status: "starting" });
        sendOutputs(ws, [
          { type: "message", content: "[lsmart-service] Starting simulation" },
        ]);
        void runSimulation(ws, config, mapPath);
      },
    },
  });

  console.log(`[lsmart-service] Running on http://localhost:${port}`);
  console.log(`[lsmart-service] Configuration: ${JSON.stringify(config)}`);
  return server;
}

if (import.meta.main) {
  try {
    const parsed = parseRunConfig(Bun.argv.slice(2));
    if (parsed.help) {
      console.log(HELP);
    } else {
      startServer(parsed.config);
    }
  } catch (error) {
    console.error(
      `[lsmart-service] ${error instanceof Error ? error.message : String(error)}`
    );
    console.error("Run lsmart-viz --help for supported options.");
    process.exit(2);
  }
}
