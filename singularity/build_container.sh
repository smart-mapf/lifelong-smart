#!/usr/bin/env bash

set -euo pipefail

usage() {
    echo "Usage: $0 [--local] [DOCKER_IMAGE]"
    echo
    echo "Build from Docker Hub (default):"
    echo "  $0 [lunjohnzhang/lsmart:<VERSION>]"
    echo
    echo "Convert an image in the local Docker daemon:"
    echo "  $0 --local [lsmart:<VERSION>]"
}

bootstrap="docker"
if [ "${1:-}" = "--local" ]; then
    bootstrap="docker-daemon"
    shift
elif [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    usage
    exit 0
fi

if [ "$#" -gt 1 ]; then
    usage >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(<"${project_root}/VERSION")"
docker_image="${1:-lunjohnzhang/lsmart:${version}}"
definition="${project_root}/singularity/container.def"
output="${project_root}/singularity/container.sif"

if [[ ! "${version}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "VERSION must use X.Y.Z format; found: ${version}" >&2
    exit 1
fi

if command -v apptainer >/dev/null 2>&1; then
    container_runtime="apptainer"
elif command -v singularity >/dev/null 2>&1; then
    container_runtime="singularity"
else
    echo "Error: install Apptainer or Singularity before building the SIF." >&2
    exit 1
fi

echo "Building ${output} from ${bootstrap} image ${docker_image}"
"${container_runtime}" build --force \
    --build-arg "bootstrap=${bootstrap}" \
    --build-arg "docker_image=${docker_image}" \
    --build-arg "version=${version}" \
    "${output}" "${definition}"
