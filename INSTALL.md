## Installation

1. Install Argos 3. Please refer to this [Link](https://www.argos-sim.info/core.php) for instruction.

   You can verify the correctness of the compilation by running:

   ```bash
   argos3 --version
   ```
2. Install RPC
   This repo requires [RPC](https://github.com/rpclib/rpclib) for communication
   between server and clients.
   Please install rpc using:

   ```bash
   bash compile.sh rpclib
   ```
3. Install the minimal Python dependencies for `run_lifelong.py`.

   ```bash
   python -m pip install -r requirement.txt
   ```
4. Compile client.

   ```bash
   bash compile.sh client
   ```

   To produce debuggable code (slow), type:

   ```bash
   cd client
   cmake -DCMAKE_BUILD_TYPE=Debug ..
   make
   cd ..
   ```
5. Compile server.

   ```bash
   bash compile.sh server
   ```
6. Compile MAPF planner. For now we support PBS and RHCR.

   ```bash
   bash compile.sh pbs
   bash compile.sh rhcr
   ```

Alternatively, you may compile rpc, server, client, and MAPF planners using:

```bash
bash compile.sh all
```

While developing, you may compile server, client, and MAPF planners using:

```bash
bash compile.sh user
```

## Docker

The Docker image contains LSMART, its open-source planners, ARGoS, and the
browser visualizer. It is currently `linux/amd64`-only because the supplied
ARGoS package is `amd64`-only.

### Install Docker and build the image

1. Install and start Docker:

   - macOS or Windows: install
     [Docker Desktop](https://docs.docker.com/desktop/).
   - Linux: install
     [Docker Engine](https://docs.docker.com/engine/install/).

   Verify that the Docker daemon and Buildx are available:

   ```bash
   docker version
   docker buildx version
   ```

   If Linux reports permission denied for `/var/run/docker.sock`, follow
   Docker's
   [Linux post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/)
   or run the Docker commands with `sudo`.
2. Download `argos3_simulator-3.0.0-x86_64-beta59.deb` from
   [ARGoS](https://www.argos-sim.info/core.php) and place it in the repository
   root. This untracked file is required during the build.
3. From the repository root, build and install the local image:

   ```bash
   docker buildx build \
     --platform linux/amd64 \
     --load \
     -t lsmart:0.1.0 .
   ```

On Apple Silicon, Docker uses `linux/amd64` emulation, so builds and
simulations are slower than on an `amd64` machine.

### Run with browser visualization

Start the default visualization server:

```bash
docker run --rm --init \
  -p 3000:3000 \
  lsmart:0.1.0
```

Then open [http://localhost:3000](http://localhost:3000), review the effective
configuration, and select **Start Simulation**.

To configure the simulation, pass `lsmart-viz` options after the image name:

```bash
docker run --rm --init \
  -p 3000:3000 \
  lsmart:0.1.0 \
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
docker run --rm lsmart:0.1.0 lsmart-viz --help
```

To continue until `sim_duration` even if LSMART detects congestion, add:

```bash
--stop_at_congestion=false
```

Bundled maps use repository-relative paths. To visualize a custom map, mount
it read-only and pass its container path:

```bash
docker run --rm --init \
  -p 3000:3000 \
  -v "$PWD/custom.json:/workspace/custom.json:ro" \
  lsmart:0.1.0 \
  lsmart-viz \
    --map_filepath=/workspace/custom.json \
    --num_agents=20
```

### Run without browser visualization

Override the default command with `run_lifelong.py`. No port mapping is needed.
Mount a host directory at `/workspace` to retain the results:

```bash
mkdir -p results

docker run --rm --init \
  -v "$PWD/results:/workspace" \
  lsmart:0.1.0 \
  python3 run_lifelong.py \
    maps/kiva_large_w_mode.json \
    --headless True \
    --container True \
    --num_agents 10 \
    --sim_duration 300 \
    --save_stats True \
    --stats_name /workspace/stats.json
```

The simulation runs entirely in the terminal and writes its result to
`results/stats.json`. Keep `--container True` when invoking
`run_lifelong.py` inside this image.

### Stop and remove

Press `Ctrl-C` to stop a foreground container. For a background container,
use:

```bash
docker ps
docker stop CONTAINER_ID
```

The examples use `--rm`, so stopped containers are removed automatically.
To remove the locally built image:

```bash
docker image rm lsmart:0.1.0
```

## Singularity

You can also build and run LSMART inside a [Singularity](https://github.com/sylabs/singularity) container.

The container build expects **Argos3** to already exist in the repository
root because `singularity/container.def` copies it into the image:

1. `argos3_simulator-3.0.0-x86_64-beta59.deb`: Download it from [Argos3](https://www.argos-sim.info/core.php).
2. Install CPLEX at `CPLEX_Studio2210/`: If it exists in the repository root, the container
   build also copies it into the image and compiles the MASS planner. If it is missing, the
   container still builds, but skips MASS.

Build the container with:

```bash
bash singularity/build_container.sh
```

This produces `singularity/container.sif`. The build script first creates a
writable sandbox, runs it once to compile LSMART inside the container, and
then packs the result into the final `.sif` image. During the build, the
container also installs the Python packages from `requirement.txt`.

To run a headless simulation from the container:

```bash
singularity exec --cleanenv \
  singularity/container.sif \
  python run_lifelong.py maps/kiva_large_w_mode.json --headless True --screen 0 --num_agents 10 --save_stats True --rotation False --planner_invoke_policy default --sim_window_tick 10 --planner RHCR --backup_solver PIBT --task_assigner_type windowed --container True
```
