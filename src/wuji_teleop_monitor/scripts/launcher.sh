#!/usr/bin/env bash
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$REPO_ROOT/tools/env.sh"
case "${1:-monitor}" in
    monitor|brake) exec ros2 run wuji_teleop_monitor "${1:-monitor}" ;;
    *) echo "Usage: $0 [monitor|brake]" >&2; exit 2 ;;
esac
