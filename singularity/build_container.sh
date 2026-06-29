#!/bin/bash

set -euo pipefail

sudo singularity build --force --disable-cache --sandbox singularity/container singularity/container.def
sudo singularity run --writable singularity/container
sudo singularity build --force singularity/container.sif singularity/container
