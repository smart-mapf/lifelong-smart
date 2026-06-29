import { spawn, type Subprocess } from "bun";
import { load } from "js-yaml";
import { readFileSync, existsSync } from "fs";
import { createServer } from "net";
import { join, resolve } from "path";
import type { ServerWebSocket } from "bun";

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
  map_contents: unknown;
  map_format: "lsmart-json";
  map_path: string;
  agent_count: number;
  ticks_per_second: number;
  sim_duration: number;
  planner: string;
  task_assigner_type: string;
  backup_solver: string;
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
  proc: Subprocess;
  cancelled: boolean;
  simulatorPort: number;
};

// ─── Default Run Config ─────────────────────────────────────────────────────

const REPO_ROOT = resolve(import.meta.dir, "../..");

const DEFAULT_RUN = {
  mapPath: "maps/kiva_large_w_mode.json",
  numAgents: 50,
  planner: "RHCR",
  plannerInvokePolicy: "default",
  taskAssignerType: "windowed",
  backupSolver: "PIBT",
  simDuration: 600,
  simWindowTick: 20,
  ticksPerSecond: 10,
  velocity: 200.0,
  seed: 42,
  rotation: false,
};

const activeRuns = new WeakMap<ServerWebSocket<unknown>, ActiveRun>();

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
  config: typeof DEFAULT_RUN
) {
  const mapPath = join(REPO_ROOT, config.mapPath);
  if (!existsSync(mapPath)) {
    ws.send(
      JSON.stringify({
        type: "error",
        error: `Map file not found: ${config.mapPath}`,
      })
    );
    return;
  }

  // Read map contents for meta event
  const mapContents = JSON.parse(readFileSync(mapPath, "utf-8"));

  // Send meta event first
  const meta: MetaEvent = {
    type: "meta",
    map_contents: mapContents,
    map_format: "lsmart-json",
    map_path: config.mapPath,
    agent_count: config.numAgents,
    ticks_per_second: config.ticksPerSecond,
    sim_duration: config.simDuration,
    planner: config.planner,
    task_assigner_type: config.taskAssignerType,
    backup_solver: config.backupSolver,
  };
  ws.send(JSON.stringify(meta));

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
    return;
  }
  sendOutputs(ws, [
    {
      type: "message",
      content: `[lsmart-service] Using simulator RPC port ${simulatorPort}`,
    },
  ]);

  if (ws.readyState !== WebSocket.OPEN) {
    return;
  }

  // Build command
  const extVizPluginDir = join(
    REPO_ROOT,
    "plugins/visualizers/external_visualizer/build"
  );
  const args = [
    "run_lifelong.py",
    `--map_filepath=${config.mapPath}`,
    `--num_agents=${config.numAgents}`,
    `--planner=${config.planner}`,
    `--planner_invoke_policy=${config.plannerInvokePolicy}`,
    `--task_assigner_type=${config.taskAssignerType}`,
    `--backup_solver=${config.backupSolver}`,
    `--sim_duration=${config.simDuration}`,
    `--sim_window_tick=${config.simWindowTick}`,
    `--ticks_per_second=${config.ticksPerSecond}`,
    `--velocity=${config.velocity}`,
    `--seed=${config.seed}`,
    `--port_num=${simulatorPort}`,
    `--external_visualization=True`,
    `--headless=False`,
    `--screen=0`,
    `--save_stats=False`,
  ];

  let proc: Subprocess;
  try {
    proc = spawn({
      cmd: ["/home/yulun/miniconda3/envs/surrogate_ggo/bin/python", ...args],
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
    return;
  }

  const runState: ActiveRun = {
    proc,
    cancelled: false,
    simulatorPort,
  };
  activeRuns.set(ws, runState);

  if (ws.readyState !== WebSocket.OPEN) {
    runState.cancelled = true;
    try {
      proc.kill();
    } catch {}
    activeRuns.delete(ws);
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
    activeRuns.delete(ws);
    try {
      if (!exited) {
        proc.kill();
      }
    } catch {}
  }
}

// ─── HTTP + WebSocket Server ────────────────────────────────────────────────

const PORT = parseInt(process.env.PORT || "3000");
const frontendDevServerUrl = process.env.LSMART_VISUALIZER_DEV_URL?.replace(
  /\/+$/,
  ""
);

// Try to serve static frontend files
const staticDir = join(import.meta.dir, "../lsmart-visualiser/dist");

Bun.serve({
  port: PORT,
  async fetch(req, server) {
    const url = new URL(req.url);

    // Only the simulation stream should upgrade to websocket.
    if (url.pathname === "/ws") {
      if (server.upgrade(req)) {
        return;
      }

      return new Response("WebSocket upgrade required", { status: 426 });
    }

    if (frontendDevServerUrl) {
      return Response.redirect(
        `${frontendDevServerUrl}${url.pathname}${url.search}`,
        307
      );
    }

    // Serve static files
    let path = url.pathname;
    if (path === "/") path = "/index.html";

    const filePath = join(staticDir, path);
    if (existsSync(filePath)) {
      return new Response(Bun.file(filePath));
    }

    return new Response("Not Found", { status: 404 });
  },
  websocket: {
    open(ws) {
      console.log("[lsmart-service] Client connected, starting simulation...");
      void runSimulation(ws, DEFAULT_RUN);
    },
    close(ws) {
      console.log("[lsmart-service] Client disconnected");
      const runState = activeRuns.get(ws);
      if (!runState) {
        return;
      }

      runState.cancelled = true;
      try {
        runState.proc.kill();
      } catch {}
      activeRuns.delete(ws);
    },
    message(ws, message) {
      // Handle client messages if needed (e.g., custom run config)
    },
  },
});

console.log(`[lsmart-service] Running on http://localhost:${PORT}`);
