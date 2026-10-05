#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(<"${project_root}/VERSION")"
repository="${1:-lunjohnzhang/lsmart}"

if [[ ! "${version}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "VERSION must use X.Y.Z format; found: ${version}" >&2
    exit 1
fi

docker buildx build \
    --platform linux/amd64 \
    --push \
    --file "${project_root}/docker/Dockerfile" \
    --build-arg "LSMART_VERSION=${version}" \
    --tag "${repository}:${version}" \
    --tag "${repository}:latest" \
    "${project_root}"
