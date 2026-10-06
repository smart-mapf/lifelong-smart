import { spawn, type Subprocess } from "bun";
import { resolve } from "path";

const SERVICE_DIR = import.meta.dir;
const VISUALISER_DIR = resolve(SERVICE_DIR, "../lsmart-visualiser");
const FRONTEND_DEV_URL = "http://127.0.0.1:5173";
const serviceArgs = Bun.argv.slice(2).filter((arg) => arg !== "--");

function startProcess(
  cmd: string[],
  cwd: string,
  env?: Record<string, string>
): Subprocess {
  return spawn({
    cmd,
    cwd,
    stdout: "inherit",
    stderr: "inherit",
    stdin: "inherit",
    env: {
      ...process.env,
      ...env,
    },
  });
}

const backend = startProcess(
  ["bun", "--watch", "index.ts", ...serviceArgs],
  SERVICE_DIR,
  { LSMART_VISUALIZER_DEV_URL: FRONTEND_DEV_URL }
);
const frontend = startProcess(["bun", "run", "dev"], VISUALISER_DIR, {
  VITE_LSMART_WS_URL: "ws://127.0.0.1:3000/ws",
});

const children = [backend, frontend];
let shuttingDown = false;

function shutdown(signal?: NodeJS.Signals) {
  if (shuttingDown) return;

  shuttingDown = true;
  for (const child of children) {
    try {
      child.kill(signal ?? "SIGTERM");
    } catch {}
  }
}

for (const signal of ["SIGINT", "SIGTERM"] as const) {
  process.on(signal, () => shutdown(signal));
}

console.log("[lsmart-service] Running from the native source checkout");
console.log("[lsmart-service] Backend: http://127.0.0.1:3000");
console.log(`[lsmart-service] Frontend: ${FRONTEND_DEV_URL}`);

const winner = await Promise.race([
  backend.exited.then((code) => ({ name: "backend", code })),
  frontend.exited.then((code) => ({ name: "frontend", code })),
]);

shutdown();
await Promise.allSettled(children.map((child) => child.exited));

process.exitCode = winner.code ?? 1;
console.error(
  `[lsmart-service] ${winner.name} exited with code ${winner.code ?? "unknown"}`
);
