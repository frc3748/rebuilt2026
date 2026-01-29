#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=${IMAGE_NAME:-frc-rebuilt2026}

docker build -t "$IMAGE_NAME" ./.portable-env
