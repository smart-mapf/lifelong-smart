FROM oven/bun:1.3.14 AS bun

FROM ubuntu:20.04 AS build

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
    && bun run build

FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG LSMART_VERSION=dev

ENV TZ=America/New_York \
    PROJECT_ROOT=/usr/project \
    PYTHONPATH=/usr/project \
    ARGOS_PLUGIN_PATH=/usr/project/plugins/visualizers/external_visualizer/build \
    LSMART_CONTAINER=true \
    OPENBLAS_NUM_THREADS=1 \
    MALLOC_TRIM_THRESHOLD_=0 \
    PORT=3000

COPY --from=bun /usr/local/bin/bun /usr/local/bin/bun

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        freeglut3 \
        libc-bin \
        libc6 \
        libboost-filesystem1.71.0 \
        libboost-program-options1.71.0 \
        libfreeimage3 \
        libfreeimageplus3 \
        libglu1-mesa \
        liblua5.3-0 \
        libqt5widgets5 \
        libspdlog1 \
        libxi6 \
        libxmu6 \
        python-is-python3 \
        python3.8 \
        tzdata \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/project

COPY LICENSE.txt /usr/share/doc/lsmart/LICENSE.txt
COPY ArgosConfig ./ArgosConfig
COPY maps ./maps
COPY run_lifelong.py run_lifelong_kwargs.py ./
COPY web/lsmart-service ./web/lsmart-service
COPY --from=build /usr/bin/argos3 /usr/bin/argos3
COPY --from=build /usr/lib/argos3 /usr/lib/argos3
COPY --from=build /usr/doc/argos3/licenses /usr/share/doc/argos3/licenses
COPY --from=build /etc/ld.so.conf.d/argos3.conf /etc/ld.so.conf.d/argos3.conf
COPY --from=build /usr/local/lib/python3.8/dist-packages /usr/local/lib/python3.8/dist-packages
COPY --from=build /usr/project/client/build ./client/build
COPY --from=build /usr/project/server/build/ExecutionManager ./server/build/ExecutionManager
COPY --from=build /usr/project/planner/PBS/build/pbs ./planner/PBS/build/pbs
COPY --from=build /usr/project/planner/Transient_PBS/build/tpbs ./planner/Transient_PBS/build/tpbs
COPY --from=build /usr/project/planner/RHCR/build/lifelong ./planner/RHCR/build/lifelong
COPY --from=build /usr/project/plugins/visualizers/external_visualizer/build/libexternal_visualizer.so ./plugins/visualizers/external_visualizer/build/libexternal_visualizer.so
COPY --from=build /usr/project/web/lsmart-visualiser/dist ./web/lsmart-visualiser/dist

RUN useradd --create-home --uid 1000 --shell /bin/bash lsmart \
    && mkdir -p /workspace \
    && ldconfig \
    && chmod +x /usr/project/web/lsmart-service/index.ts \
    && ln -s /usr/project/web/lsmart-service/index.ts /usr/local/bin/lsmart-viz \
    && chown -R lsmart:lsmart /usr/project /workspace

LABEL org.opencontainers.image.title="LSMART" \
      org.opencontainers.image.version="${LSMART_VERSION}" \
      org.opencontainers.image.source="https://github.com/smart-mapf/lifelong-smart" \
      org.opencontainers.image.licenses="MIT"

USER lsmart

EXPOSE 3000

CMD ["lsmart-viz"]
