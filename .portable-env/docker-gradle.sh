#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=${IMAGE_NAME:-frc-rebuilt2026}

DISPLAY_ARG=()
if [ -n "${DISPLAY:-}" ]; then
  DISPLAY_ARG=(-e DISPLAY="$DISPLAY" -e LIBGL_ALWAYS_INDIRECT=1 -e QT_X11_NO_MITSHM=1)
fi

TTY_ARG=()
if [ -t 1 ]; then
  TTY_ARG=(-t)
fi

set +u
exec docker run --rm -i \
  "${TTY_ARG[@]}" \
  -v "${PWD}:/workspace" \
  -v "${HOME}/.gradle:/home/gradle/.gradle" \
  -w /workspace \
  "${DISPLAY_ARG[@]}" \
  "$IMAGE_NAME" \
  ./gradlew "$@"
