import { useEffect, useRef, useState } from "react";
import { useAtomValue, useSetAtom } from "jotai";
import {
  useSimulation,
  metaAtom,
  statsAtom,
  logsAtom,
  errorAtom,
  framesAtom,
  agentCountAtom,
  agentStatesAtom,
  currentFrameAtom,
  playingAtom,
  speedAtom,
} from "./state";
import Scene from "./Scene";

const STATUS_COLORS: Record<string, string> = {
  unknown: "#666",
  initialized: "#ffcc00",
  active: "#00ff88",
  idle: "#4488ff",
  finished: "#888",
};

function ControlPanel({
  started,
  setStarted,
}: {
  started: boolean;
  setStarted: (value: boolean) => void;
}) {
  const meta = useAtomValue(metaAtom);
  const stats = useAtomValue(statsAtom);
  const logs = useAtomValue(logsAtom);
  const error = useAtomValue(errorAtom);
  const frames = useAtomValue(framesAtom);
  const agentCount = useAtomValue(agentCountAtom);
  const agentStates = useAtomValue(agentStatesAtom);
  const currentFrame = useAtomValue(currentFrameAtom);
  const playing = useAtomValue(playingAtom);
  const speed = useAtomValue(speedAtom);
  const setPlaying = useSetAtom(playingAtom);
  const setSpeed = useSetAtom(speedAtom);
  const setCurrentFrame = useSetAtom(currentFrameAtom);
  const [minimized, setMinimized] = useState(false);
  const bottomRef = useRef<HTMLDivElement>(null);
  const maxFrame = Math.max(frames.length - 1, 0);

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [logs]);

  if (minimized) {
    return (
      <div style={styles.panelDock}>
        <div style={styles.panelCollapsed}>
          <span style={styles.collapsedTitle}>LSMART Visualizer</span>
          <button
            type="button"
            onClick={() => setMinimized(false)}
            style={styles.iconBtn}
          >
            ▸
          </button>
        </div>
      </div>
    );
  }

  return (
    <div style={styles.panelDock}>
      <div style={styles.panel}>
        <div style={styles.panelHeader}>
          <div>
            <div style={styles.panelTitle}>LSMART Visualizer</div>
            <div style={styles.panelSubtitle}>
              {meta?.map_path || "Waiting for simulation metadata"}
            </div>
          </div>
          <button
            type="button"
            onClick={() => setMinimized(true)}
            style={styles.iconBtn}
          >
            ▾
          </button>
        </div>

        <div style={styles.panelBody}>
          <details open style={styles.section}>
            <summary style={styles.sectionTitle}>Run</summary>
            <div style={styles.sectionBody}>
              <button
                type="button"
                onClick={() => setStarted(true)}
                disabled={started}
                style={
                  started
                    ? { ...styles.primaryBtn, ...styles.disabledBtn }
                    : styles.primaryBtn
                }
              >
                {started ? "Simulation Started" : "Start Simulation"}
              </button>
              {error && <div style={styles.errorText}>{error}</div>}
            </div>
          </details>

          <details open style={styles.section}>
            <summary style={styles.sectionTitle}>Scenario</summary>
            <div style={styles.sectionBody}>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Map</span>
                <span style={styles.infoValue}>{meta?.map_path || "—"}</span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Planner</span>
                <span style={styles.infoValue}>{meta?.planner || "—"}</span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Assigner</span>
                <span style={styles.infoValue}>
                  {meta?.task_assigner_type || "—"}
                </span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Agents</span>
                <span style={styles.infoValue}>{agentCount}</span>
              </div>
            </div>
          </details>

          <details open style={styles.section}>
            <summary style={styles.sectionTitle}>Playback</summary>
            <div style={styles.sectionBody}>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Frame</span>
                <span style={styles.infoValue}>
                  {currentFrame} / {maxFrame}
                </span>
              </div>
              <input
                type="range"
                min={0}
                max={maxFrame}
                value={currentFrame}
                onChange={(e) => setCurrentFrame(Number(e.target.value))}
                style={styles.slider}
              />
              <div style={styles.controlsRow}>
                <button
                  type="button"
                  onClick={() => setPlaying(!playing)}
                  style={styles.primaryBtn}
                >
                  {playing ? "Pause" : "Play"}
                </button>
                <select
                  value={speed}
                  onChange={(e) => setSpeed(Number(e.target.value))}
                  style={styles.select}
                >
                  <option value={0.5}>0.5x</option>
                  <option value={1}>1x</option>
                  <option value={2}>2x</option>
                  <option value={5}>5x</option>
                  <option value={10}>10x</option>
                </select>
              </div>
            </div>
          </details>

          <details open style={styles.section}>
            <summary style={styles.sectionTitle}>Statistics</summary>
            <div style={styles.sectionBody}>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Finished tasks</span>
                <span style={styles.infoValue}>
                  {stats?.total_finished_tasks ?? "—"}
                </span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Throughput</span>
                <span style={styles.infoValue}>
                  {stats?.throughput != null
                    ? `${Number(stats.throughput).toFixed(2)} tasks/s`
                    : "—"}
                </span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>CPU runtime</span>
                <span style={styles.infoValue}>
                  {stats?.cpu_runtime != null
                    ? `${Number(stats.cpu_runtime).toFixed(2)} s`
                    : "—"}
                </span>
              </div>
            </div>
          </details>

          <details open style={styles.section}>
            <summary style={styles.sectionTitle}>Agents</summary>
            <div style={styles.listBox}>
              {agentCount === 0 ? (
                <div style={styles.emptyText}>Waiting for agent state...</div>
              ) : (
                Array.from({ length: agentCount }, (_, i) => {
                  const state = agentStates[i];
                  const status = state?.status || "unknown";
                  const progress = state?.progress;

                  return (
                    <div key={i} style={styles.agentRow}>
                      <span
                        style={{
                          ...styles.dot,
                          backgroundColor:
                            STATUS_COLORS[status] || STATUS_COLORS.unknown,
                        }}
                      />
                      <span style={styles.agentLabel}>
                        Agent {i} - {status}
                        {progress
                          ? ` (${progress.finished}/${progress.total})`
                          : ""}
                      </span>
                    </div>
                  );
                })
              )}
            </div>
          </details>

          <details open style={styles.section}>
            <summary style={styles.sectionTitle}>Logs</summary>
            <div style={styles.logBox}>
              {logs.length === 0 ? (
                <div style={styles.emptyText}>No logs yet.</div>
              ) : (
                logs.map((log, i) => (
                  <div key={i} style={styles.logLine}>
                    {log}
                  </div>
                ))
              )}
              <div ref={bottomRef} />
            </div>
          </details>
        </div>
      </div>
    </div>
  );
}

export default function App() {
  const [started, setStarted] = useState(false);

  useSimulation(started);

  return (
    <div style={styles.container}>
      <Scene />
      <div style={styles.overlay}>
        <ControlPanel started={started} setStarted={setStarted} />
      </div>
    </div>
  );
}

const styles: Record<string, React.CSSProperties> = {
  container: {
    position: "relative",
    width: "100vw",
    height: "100vh",
    overflow: "hidden",
  },
  overlay: {
    position: "absolute",
    inset: 0,
    pointerEvents: "none",
  },
  panelDock: {
    position: "absolute",
    top: 16,
    right: 16,
    bottom: 16,
    pointerEvents: "auto",
    display: "flex",
    alignItems: "flex-start",
    justifyContent: "flex-end",
  },
  panel: {
    width: "min(380px, calc(100vw - 32px))",
    maxHeight: "100%",
    display: "flex",
    flexDirection: "column",
    background: "rgba(18, 22, 28, 0.94)",
    color: "#d5d9e3",
    border: "1px solid rgba(255,255,255,0.08)",
    borderRadius: "14px",
    boxShadow: "0 18px 48px rgba(0,0,0,0.34)",
    backdropFilter: "blur(10px)",
    overflow: "hidden",
    fontFamily:
      '"SFMono-Regular", "ui-monospace", "Cascadia Code", "Source Code Pro", monospace',
  },
  panelCollapsed: {
    minWidth: "220px",
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    gap: "12px",
    padding: "12px 14px",
    background: "rgba(18, 22, 28, 0.94)",
    color: "#d5d9e3",
    border: "1px solid rgba(255,255,255,0.08)",
    borderRadius: "14px",
    boxShadow: "0 18px 48px rgba(0,0,0,0.34)",
    backdropFilter: "blur(10px)",
  },
  collapsedTitle: {
    fontSize: "13px",
    letterSpacing: "0.02em",
  },
  panelHeader: {
    display: "flex",
    alignItems: "flex-start",
    justifyContent: "space-between",
    gap: "12px",
    padding: "14px 16px 12px",
    borderBottom: "1px solid rgba(255,255,255,0.08)",
    background:
      "linear-gradient(180deg, rgba(58,64,80,0.78), rgba(25,30,40,0.2))",
  },
  panelTitle: {
    fontSize: "16px",
    color: "#f5f7fb",
  },
  panelSubtitle: {
    marginTop: "4px",
    fontSize: "11px",
    color: "#8d96aa",
    wordBreak: "break-word",
  },
  iconBtn: {
    background: "rgba(255,255,255,0.06)",
    color: "#d5d9e3",
    border: "1px solid rgba(255,255,255,0.08)",
    borderRadius: "8px",
    width: "32px",
    height: "32px",
    cursor: "pointer",
    fontSize: "14px",
  },
  panelBody: {
    padding: "12px",
    overflowY: "auto",
    display: "flex",
    flexDirection: "column",
    gap: "10px",
  },
  section: {
    background: "rgba(255,255,255,0.02)",
    border: "1px solid rgba(255,255,255,0.06)",
    borderRadius: "10px",
    padding: "0 12px 12px",
  },
  sectionTitle: {
    cursor: "pointer",
    listStyle: "none",
    padding: "12px 0",
    fontSize: "12px",
    color: "#eef2fb",
  },
  sectionBody: {
    display: "flex",
    flexDirection: "column",
    gap: "10px",
  },
  infoRow: {
    display: "flex",
    justifyContent: "space-between",
    gap: "12px",
    fontSize: "12px",
    lineHeight: 1.5,
  },
  infoLabel: {
    color: "#8d96aa",
  },
  infoValue: {
    color: "#f5f7fb",
    textAlign: "right",
    wordBreak: "break-word",
  },
  slider: {
    width: "100%",
    accentColor: "#2383ff",
  },
  controlsRow: {
    display: "flex",
    gap: "10px",
  },
  primaryBtn: {
    width: "100%",
    background: "#2383ff",
    color: "#ffffff",
    border: "none",
    borderRadius: "8px",
    padding: "11px 14px",
    cursor: "pointer",
    fontFamily: "inherit",
    fontSize: "13px",
  },
  disabledBtn: {
    opacity: 0.55,
    cursor: "default",
  },
  select: {
    minWidth: "88px",
    background: "#30374a",
    color: "#f5f7fb",
    border: "1px solid rgba(255,255,255,0.08)",
    borderRadius: "8px",
    padding: "10px 12px",
    fontFamily: "inherit",
    fontSize: "12px",
  },
  errorText: {
    color: "#ff7b7b",
    fontSize: "12px",
    lineHeight: 1.5,
  },
  listBox: {
    maxHeight: "180px",
    overflowY: "auto",
    display: "flex",
    flexDirection: "column",
    gap: "6px",
    paddingTop: "2px",
  },
  agentRow: {
    display: "flex",
    alignItems: "center",
    gap: "8px",
    fontSize: "12px",
  },
  dot: {
    width: "8px",
    height: "8px",
    borderRadius: "999px",
    flexShrink: 0,
  },
  agentLabel: {
    whiteSpace: "nowrap",
    overflow: "hidden",
    textOverflow: "ellipsis",
  },
  logBox: {
    maxHeight: "220px",
    overflowY: "auto",
    display: "flex",
    flexDirection: "column",
    gap: "4px",
    paddingTop: "2px",
  },
  logLine: {
    color: "#aeb6c8",
    fontSize: "11px",
    lineHeight: 1.45,
    wordBreak: "break-word",
  },
  emptyText: {
    color: "#6f7890",
    fontSize: "12px",
  },
};
