#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=${IMAGE_NAME:-frc-rebuilt2026}

exec docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -v "${PWD}:/workspace" \
  -v "${HOME}/.gradle:/home/gradle/.gradle" \
  -w /workspace \
  "$IMAGE_NAME" \
  bash -lc 'mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix
  Xvfb :1 -screen 0 1280x720x24 &
  export DISPLAY=:1
  fluxbox &
  x11vnc -display :1 -forever -shared -rfbport 5900 -noxdamage -nowf -nowcr -nolookup &
  /usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &
  ./gradlew simulateJavaRelease -Psimgui -Djava.awt.headless=false'
