# Installation

Choose the installation method that fits your use case, listed from easiest
to most involved:

1. Pull the published Docker image and run it.
2. Build the Docker image from source.
3. Convert the Docker image to Singularity/Apptainer.
4. Build the entire project from source.

The container image is currently `linux/amd64` because the supplied ARGoS
package is `amd64`-only. Docker Desktop uses emulation on Apple Silicon, so
simulations run more slowly than on an `amd64` machine.

## 1. Pull and run the Docker image

Install and start Docker:

- macOS or Windows: install
  [Docker Desktop](https://docs.docker.com/desktop/).
- Linux: install
  [Docker Engine](https://docs.docker.com/engine/install/).

Verify Docker, then pull the published LSMART image:

```bash
docker version
docker pull --platform linux/amd64 lunjohnzhang/lsmart:latest
```

Run the pull command again before a workshop or experiment to refresh a
cached image. Alternatively, add `--pull=always` to `docker run` when you
always want Docker to check for a newer `latest` image.

If Linux reports permission denied for `/var/run/docker.sock`, follow
Docker's
[Linux post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/)
or run Docker commands with `sudo`.

### Run with browser visualization

Start the default visualization server:

```bash
docker run --rm --init \
  --platform linux/amd64 \
  -p 3000:3000 \
  lunjohnzhang/lsmart:latest
```

Open [http://localhost:3000](http://localhost:3000), review the effective
configuration, and select **Start Simulation**.

To configure the simulation, pass `lsmart-viz` options after the image name:

```bash
docker run --rm --init \
  --platform linux/amd64 \
  -p 3000:3000 \
  lunjohnzhang/lsmart:latest \
  lsmart-viz \
    --map_filepath=maps/kiva_large_w_mode.json \
    --num_agents=20 \
    --planner=RHCR \
    --task_assigner_type=windowed \
    --sim_duration=600 \
    --rotation=false \
    --seed=42
```

List all browser-visualization options with:

```bash
docker run --rm --platform linux/amd64 \
  lunjohnzhang/lsmart:latest \
  lsmart-viz --help
```

To continue until `sim_duration` even if LSMART detects congestion, add:

```bash
--stop_at_congestion=false
```

Bundled maps use repository-relative paths. To visualize a custom map, mount
it read-only and pass its container path:

```bash
docker run --rm --init \
  --platform linux/amd64 \
  -p 3000:3000 \
  -v "$PWD/custom.json:/workspace/custom.json:ro" \
  lunjohnzhang/lsmart:latest \
  lsmart-viz \
    --map_filepath=/workspace/custom.json \
    --num_agents=20
```

### Run without visualization

Override the default command with `run_lifelong.py`. No port mapping is needed.
Mount a host directory at `/workspace` to retain the results:

```bash
mkdir -p results

docker run --rm --init \
  --platform linux/amd64 \
  -v "$PWD/results:/workspace" \
  lunjohnzhang/lsmart:latest \
  python3 run_lifelong.py \
    maps/kiva_large_w_mode.json \
    --visualizer none \
    --container True \
    --num_agents 10 \
    --sim_duration 300 \
    --save_stats True \
    --stats_name /workspace/stats.json
```

The simulation writes its result to `results/stats.json`. Keep
`--container True` when invoking `run_lifelong.py` inside the image.

### Run with the native ARGoS visualizer

The native visualizer requires an X11 display. On Linux, permit the container
to use the current display and mount the X11 socket:

```bash
xhost +local:docker
docker run --rm --init \
  --platform linux/amd64 \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  lunjohnzhang/lsmart:latest \
  python3 run_lifelong.py \
    maps/kiva_large_w_mode.json \
    --visualizer argos \
    --container True \
    --num_agents 10
xhost -local:docker
```

Docker Desktop on macOS and Windows does not provide an X server. Use the web
visualizer there, or install and configure a separate host X server.

### Stop and remove

Press `Ctrl-C` to stop a foreground container. For a background container,
use:

```bash
docker ps
docker stop CONTAINER_ID
```

The examples use `--rm`, so stopped containers are removed automatically.
To remove the downloaded image:

```bash
docker image rm lunjohnzhang/lsmart:latest
```

## 2. Build the Docker image from source

Use this method when developing LSMART or testing changes that have not been
published to Docker Hub.

1. Verify that Docker Buildx is available:

   ```bash
   docker buildx version
   ```
2. Download `argos3_simulator-3.0.0-x86_64-beta59.deb` from
   [ARGoS](https://www.argos-sim.info/core.php) and place it in the repository
   root. This untracked file is required during the build.
3. From the repository root, build the local image:

   ```bash
   bash docker/build_image.sh
   ```

   To choose another image name or tag, pass it to the script:

   ```bash
   bash docker/build_image.sh lsmart:dev
   ```

The default local tag is `lsmart:<VERSION>`, where `<VERSION>` is read from
the repository's `VERSION` file. Use that local tag in place of
`lunjohnzhang/lsmart:latest` in any Docker command above.

### Publish a release

Log in to Docker Hub, update `VERSION` to a new `X.Y.Z` release, and run:

```bash
docker login
bash docker/publish_image.sh
```

This builds once and publishes both `lunjohnzhang/lsmart:<VERSION>` and
`lunjohnzhang/lsmart:latest`. Pass another repository as the first argument
when publishing elsewhere:

```bash
bash docker/publish_image.sh YOUR_NAMESPACE/lsmart
```

Do not reuse an existing numbered release tag. Increment `VERSION` for each
release; reserve `latest` as the mutable pointer to the newest stable release.

## 3. Build the Singularity/Apptainer image

The Singularity/Apptainer image is a direct conversion of the canonical Docker
image. It does not reinstall dependencies or compile a second copy of LSMART,
so Docker and Singularity contain the same binaries, maps, browser frontend,
and default configuration.

Install either
[Apptainer](https://apptainer.org/docs/admin/main/installation.html) or a
compatible Singularity release, then convert the published Docker image:

```bash
bash singularity/build_container.sh
```

To select another published Docker image or tag, pass its reference:

```bash
bash singularity/build_container.sh lunjohnzhang/lsmart:latest
```

Developers can instead convert an image from the local Docker daemon:

```bash
bash docker/build_image.sh lsmart:dev
bash singularity/build_container.sh --local lsmart:dev
```

All commands produce `singularity/container.sif`. The definition checks for
Python, planners, ARGoS plugins, and the built browser application before the
SIF is written.

### Run Singularity without visualization

No display or network-port setup is required:

```bash
apptainer exec --cleanenv \
  singularity/container.sif \
  python3 /usr/project/run_lifelong.py \
    /usr/project/maps/kiva_large_w_mode.json \
    --visualizer none \
    --container True \
    --num_agents 10 \
    --save_stats True \
    --stats_name "$PWD/stats.json"
```

Apptainer binds the current directory by default, so `stats.json` is retained
on the host. Replace `apptainer` with `singularity` when using that command.

### Run Singularity with browser visualization

The SIF uses `lsmart-viz` as its default run command:

```bash
apptainer run singularity/container.sif \
  --num_agents=20 \
  --planner=RHCR \
  --seed=42
```

Open [http://localhost:3000](http://localhost:3000). Singularity shares the
host network, so there is no Docker-style port mapping. To select another
port:

```bash
apptainer run --env PORT=3001 singularity/container.sif
```

### Run Singularity with the native ARGoS visualizer

Run this from a graphical Linux session. Apptainer normally binds the host X11
socket automatically; pass `DISPLAY` explicitly when using `--cleanenv`:

```bash
apptainer exec --cleanenv --env DISPLAY="$DISPLAY" \
  singularity/container.sif \
  python3 /usr/project/run_lifelong.py \
    /usr/project/maps/kiva_large_w_mode.json \
    --visualizer argos \
    --container True \
    --num_agents 10
```

## 4. Build the entire project from source

Use this method when modifying native LSMART, planner, or ARGoS integration
code directly on the host.

1. Install ARGoS 3 by following the
   [ARGoS installation instructions](https://www.argos-sim.info/core.php).
   Verify the installation:

   ```bash
   argos3 --version
   ```
2. Install rpclib, which provides communication between the server and
   clients:

   ```bash
   bash compile.sh rpclib
   ```
3. Install the Python dependencies:

   ```bash
   python -m pip install -r requirement.txt
   ```
4. Compile the client, server, and supported planners:

   ```bash
   bash compile.sh client
   bash compile.sh server
   bash compile.sh pbs
   bash compile.sh rhcr
   ```

Alternatively, compile rpclib, the server, client, and planners together:

```bash
bash compile.sh all
```

For normal development rebuilds, use:

```bash
bash compile.sh user
```

To produce debuggable client code at the cost of performance:

```bash
cd client
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
cd ..
```

### Visualization modes from source

Every installation method exposes the same three visualization modes:

| `--visualizer` | Interface | How to launch from source |
|---|---|---|
| `none` | No user interface; suitable for experiments and servers | `python3 run_lifelong.py ... --visualizer none` |
| `web` | Browser UI streamed by the external ARGoS plugin | Start the Bun service as described below |
| `argos` | Native Qt/OpenGL ARGoS application | `python3 run_lifelong.py ... --visualizer argos` |

Run without visualization:

```bash
python3 run_lifelong.py \
  maps/kiva_large_w_mode.json \
  --visualizer none \
  --num_agents 10 \
  --seed 42
```

Open the native ARGoS visualizer from a graphical Linux session:

```bash
python3 run_lifelong.py \
  maps/kiva_large_w_mode.json \
  --visualizer argos \
  --num_agents 10 \
  --seed 42
```

### Run the web visualizer from source

Compile the external ARGoS visualizer plugin and install the web dependencies
with Bun 1.3.14 or newer:

```bash
bash compile.sh extviz
cd web
bun install --frozen-lockfile
```

Build the browser application and start the service:

```bash
cd lsmart-visualiser
bun run build
cd ../lsmart-service
bun run start --num_agents=20 --planner=RHCR
```

Open [http://localhost:3000](http://localhost:3000), review the effective
configuration, and select **Start Simulation**. The service detects the source
checkout and uses its locally compiled binaries.

For frontend development with hot reload, run from the repository root:

```bash
cd web/lsmart-service
bun run dev --num_agents=20 --planner=RHCR
```

Then open [http://127.0.0.1:5173](http://127.0.0.1:5173). This starts the Bun
service on port 3000 and the Vite frontend on port 5173. Stopping the launcher
with `Ctrl-C` stops both processes. Add options listed by `bun run start
--help` to either source command.

## Reproducibility across installation methods

For a fixed code revision, map, parameters, seed, and `n_threads=1`, source,
Docker, and Docker-derived Singularity runs use the same simulation logic and
produce the same deterministic simulation outcome. Compare outcome fields such
as `success`, `congested`, `total_finished_tasks`, and `throughput`. Do not use
`cpu_runtime` for equality: it is intentionally machine- and runtime-dependent.
Fine-grained per-tick diagnostic arrays can also reflect when asynchronous
planner responses arrive, so they are performance traces rather than portable
reproducibility keys.
