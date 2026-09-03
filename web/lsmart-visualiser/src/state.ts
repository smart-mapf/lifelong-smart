import { atom, useSetAtom } from "jotai";
import { useCallback, useEffect, useRef, useState } from "react";
import type { RunConfig } from "../../lsmart-service/types";

// ─── Event Types ────────────────────────────────────────────────────────────

export interface TickAgent {
  id: number;
  x: number;
  y: number;
  z: number;
  rx: number;
  ry: number;
  rz: number;
}

export interface TickEvent {
  type: "tick";
  clock: number;
  agents: TickAgent[];
}

export interface ExecProgressEvent {
  type: "exec_progress";
  agent: number;
  finished: number;
  total: number;
}

export interface StateChangeEvent {
  type: "state_change";
  agent: number;
  value: "initialized" | "active" | "idle" | "finished";
}

export interface MetaEvent {
  type: "meta";
  map_contents: {
    layout: string[];
    n_col: number;
    n_row: number;
  };
  map_format: "lsmart-json";
  map_path: string;
  agent_count: number;
  ticks_per_second: number;
  sim_duration: number;
  planner: string;
  task_assigner_type: string;
  backup_solver: string;
  effective_config: RunConfig;
}

export interface StatsEvent {
  type: "stats";
  total_finished_tasks?: number;
  throughput?: number;
  cpu_runtime?: number;
  [key: string]: unknown;
}

export type OutputEvent =
  | TickEvent
  | ExecProgressEvent
  | StateChangeEvent
  | MetaEvent
  | StatsEvent
  | { type: "message"; content: string }
  | { type: "error"; error: string };

// ─── Map State ──────────────────────────────────────────────────────────────

export interface ObstacleItem {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface MapData {
  size: { width: number; height: number };
  items: ObstacleItem[];
}

export const mapDataAtom = atom<MapData | null>(null);
export const agentCountAtom = atom<number>(0);
export const metaAtom = atom<MetaEvent | null>(null);

// ─── Frame / Playback State ─────────────────────────────────────────────────

export interface Frame {
  clock: number;
  agents: TickAgent[];
}

export const framesAtom = atom<Frame[]>([]);
export const currentFrameAtom = atom<number>(0);
export const playingAtom = atom<boolean>(false);
export const speedAtom = atom<number>(1);

// ─── Per-Agent State ────────────────────────────────────────────────────────

export interface AgentState {
  status: "unknown" | "initialized" | "active" | "idle" | "finished";
  progress: { finished: number; total: number } | null;
}

export const agentStatesAtom = atom<Record<number, AgentState>>({});

// ─── Stats & Logs ───────────────────────────────────────────────────────────

export const statsAtom = atom<StatsEvent | null>(null);
export const logsAtom = atom<string[]>([]);
export const errorAtom = atom<string | null>(null);

// ─── Map Parser ─────────────────────────────────────────────────────────────

export function parseLsmartMap(mapContents: {
  layout: string[];
  n_col: number;
  n_row: number;
}): MapData {
  const obstacles = ["@", "T"];
  const width = mapContents.n_col;
  const height = mapContents.n_row;
  const items: ObstacleItem[] = [];

  for (let r = 0; r < height; r++) {
    const row = mapContents.layout[r] || "";
    for (let c = 0; c < width; c++) {
      if (c < row.length && obstacles.includes(row[c])) {
        items.push({ x: c, y: r, width: 1, height: 1 });
      }
    }
  }

  return {
    size: { width, height },
    items,
  };
}

// ─── WebSocket Hook ─────────────────────────────────────────────────────────

export function useSimulation() {
  const setMapData = useSetAtom(mapDataAtom);
  const setAgentCount = useSetAtom(agentCountAtom);
  const setMeta = useSetAtom(metaAtom);
  const setFrames = useSetAtom(framesAtom);
  const setCurrentFrame = useSetAtom(currentFrameAtom);
  const setPlaying = useSetAtom(playingAtom);
  const setAgentStates = useSetAtom(agentStatesAtom);
  const setStats = useSetAtom(statsAtom);
  const setLogs = useSetAtom(logsAtom);
  const setError = useSetAtom(errorAtom);
  const wsRef = useRef<WebSocket | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    const defaultWsUrl = `${protocol}//${location.host}/ws`;
    const wsUrl = import.meta.env.VITE_LSMART_WS_URL || defaultWsUrl;

    let cancelled = false;
    let reconnectTimer: number | null = null;

    const connect = () => {
      if (cancelled) {
        return;
      }

      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnected(true);
        setFrames([]);
        setCurrentFrame(0);
        setAgentStates({});
        setStats(null);
        setLogs([]);
        setError(null);
        setPlaying(false);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          // Handle batch (array of events)
          const events: OutputEvent[] = Array.isArray(data) ? data : [data];

          for (const evt of events) {
            switch (evt.type) {
              case "meta":
                setMeta(evt);
                setAgentCount(evt.agent_count);
                setMapData(parseLsmartMap(evt.map_contents));
                break;

              case "tick":
                setFrames((prev) => [
                  ...prev,
                  { clock: evt.clock, agents: evt.agents },
                ]);
                break;

              case "exec_progress":
                setAgentStates((prev) => {
                  const next = { ...prev };
                  if (!next[evt.agent]) {
                    next[evt.agent] = { status: "unknown", progress: null };
                  }
                  next[evt.agent] = {
                    ...next[evt.agent],
                    progress: {
                      finished: evt.finished,
                      total: evt.total,
                    },
                  };
                  return next;
                });
                break;

              case "state_change":
                // The backend emits "finished" when its live run ends, which can
                // be well ahead of the browser replaying the buffered frames.
                // The UI derives that terminal state from playback position.
                if (evt.value === "finished") break;
                setAgentStates((prev) => {
                  const next = { ...prev };
                  if (!next[evt.agent]) {
                    next[evt.agent] = { status: "unknown", progress: null };
                  }
                  next[evt.agent] = {
                    ...next[evt.agent],
                    status: evt.value,
                  };
                  return next;
                });
                break;

              case "stats":
                setStats(evt);
                break;

              case "message":
                setLogs((prev) => [...prev.slice(-99), evt.content]);
                break;

              case "error":
                setError(evt.error);
                setPlaying(false);
                break;
            }
          }
        } catch {
          // Ignore parse errors in websocket messages
        }
      };

      ws.onerror = () => {
        if (cancelled) {
          return;
        }
        setError(`WebSocket connection error: ${wsUrl}`);
      };

      ws.onclose = () => {
        setConnected(false);
        if (cancelled) {
          return;
        }
        setPlaying(false);
        reconnectTimer = window.setTimeout(connect, 1000);
      };
    };

    connect();

    return () => {
      cancelled = true;
      if (reconnectTimer !== null) {
        window.clearTimeout(reconnectTimer);
      }
      wsRef.current?.close();
    };
  }, []);

  const startSimulation = useCallback(() => {
    if (wsRef.current?.readyState !== WebSocket.OPEN) {
      setError("Visualizer service is not connected");
      return false;
    }

    setFrames([]);
    setCurrentFrame(0);
    setAgentStates({});
    setStats(null);
    setLogs([]);
    setError(null);
    setPlaying(true);
    wsRef.current.send(JSON.stringify({ type: "start" }));
    return true;
  }, [
    setAgentStates,
    setCurrentFrame,
    setError,
    setFrames,
    setLogs,
    setPlaying,
    setStats,
  ]);

  return { connected, startSimulation };
}
