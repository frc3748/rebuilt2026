#!/usr/bin/env bash
set -euo pipefail

cmd="${1:-}"
case "$cmd" in
  build)
    exec "$(dirname "$0")/docker-build.sh"
    ;;
  gradle)
    shift
    exec "$(dirname "$0")/docker-gradle.sh" "$@"
    ;;
  sim)
    exec "$(dirname "$0")/docker-sim-gui.sh"
    ;;
  pv)
    shift
    PHOTON_JAR=${PHOTON_JAR:-photonvision-v2026.1.1-linuxx64.jar}
    PLATFORM=${PLATFORM:-linux/amd64}
    exec docker run --rm -it --platform "$PLATFORM" \
      -p 5800:5800 -p 1181-1184:1181-1184 \
      -v "${PWD}/.portable-env:/photonvision" \
      eclipse-temurin:17-jre \
      java -jar "/photonvision/$PHOTON_JAR"
    ;;
  *)
    echo "Usage: $0 {build|gradle|sim|pv} [args...]"
    exit 1
    ;;
esac
