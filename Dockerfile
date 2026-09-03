FROM oven/bun:1.3.14 AS bun

FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=America/New_York \
    PROJECT_ROOT=/usr/project \
    PYTHONPATH=/usr/project \
    ARGOS_PLUGIN_PATH=/usr/project/plugins/visualizers/external_visualizer/build \
    OPENBLAS_NUM_THREADS=1 \
    MALLOC_TRIM_THRESHOLD_=0 \
    PORT=3000

COPY --from=bun /usr/local/bin/bun /usr/local/bin/bun

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        freeglut3-dev \
        libboost-all-dev \
        libfmt-dev \
        libfreeimage-dev \
        libfreeimageplus-dev \
        liblua5.3-dev \
        libspdlog-dev \
        libxi-dev \
        libxmu-dev \
        lua5.3 \
        python-is-python3 \
        python3.8 \
        python3-pip \
        qt5-default \
        shared-mime-info \
        sudo \
        tzdata \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/project

COPY argos3_simulator-3.0.0-x86_64-beta59.deb /tmp/argos3.deb
RUN mkdir -p /etc/bash_completion.d \
    && (dpkg -i /tmp/argos3.deb \
        || (apt-get update && apt-get install -f -y)) \
    && rm /tmp/argos3.deb \
    && rm -rf /var/lib/apt/lists/*

COPY requirement.txt .
RUN python3 -m pip install --no-cache-dir -r requirement.txt

COPY web/package.json web/bun.lock ./web/
COPY web/lsmart-service/package.json ./web/lsmart-service/
COPY web/lsmart-visualiser/package.json ./web/lsmart-visualiser/
RUN cd web && bun install --frozen-lockfile

COPY . .

RUN bash compile.sh all \
    && bash compile.sh extviz \
    && cd web/lsmart-visualiser \
    && bun run build \
    && rm -rf /root/.bun/install/cache

RUN useradd --create-home --uid 1000 --shell /bin/bash lsmart \
    && mkdir -p /workspace \
    && chmod +x /usr/project/web/lsmart-service/index.ts \
    && ln -s /usr/project/web/lsmart-service/index.ts /usr/local/bin/lsmart-viz \
    && chown -R lsmart:lsmart /usr/project /workspace

USER lsmart

EXPOSE 3000

CMD ["lsmart-viz"]
