#!/usr/bin/env bash
# Copy camera and recording source; preserve existing Jetson configuration and streams.
set -euo pipefail
CAMERA_DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remote_host=nvidia@6.6.7.100
remote_path=/home/nvidia/Workspace/wuji-hand-teleop
setup_options=()
while (($#)); do
    case "$1" in
        --host) remote_host="${2:?Missing host}"; shift 2 ;;
        --path) remote_path="${2:?Missing path}"; shift 2 ;;
        --skip-system|--skip-build|--user-deps) setup_options+=("$1"); shift ;;
        --help|-h)
            cat <<'USAGE'
Usage: ./tools/deploy_camera.sh [--host USER@HOST] [--path ABSOLUTE_PATH]
                              [--skip-system] [--skip-build] [--user-deps]
Defaults: nvidia@6.6.7.100, /home/nvidia/Workspace/wuji-hand-teleop
Copies camera/writer source, setup tools, and documentation, then builds remotely.
Existing cameras.yaml and calibration YAML files are preserved.
For noninteractive deployment without sudo, use --user-deps (native prerequisites required).
Does not change or restart any camera service.
USAGE
            exit 0 ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    esac
done
[[ "$remote_path" = /* ]] || { printf '%s\n' '--path must be absolute' >&2; exit 2; }
[[ "$remote_host" != -* ]] || exit 2
printf -v mkdir_command 'mkdir -p %q %q %q %q %q' "$remote_path/src/camera" \
    "$remote_path/src/data_collection" "$remote_path/src/data_collection_interfaces" \
    "$remote_path/tools" "$remote_path/docs"
ssh "$remote_host" "$mkdir_command"
rsync -a --protect-args --exclude='__pycache__/' --exclude='*.pyc' \
    --exclude='*.yaml' \
    "$CAMERA_DEPLOY_ROOT/src/camera/" "$remote_host:$remote_path/src/camera/"
rsync -a --protect-args --exclude='__pycache__/' --exclude='*.pyc' --exclude='*.yaml' \
    "$CAMERA_DEPLOY_ROOT/src/data_collection/" "$remote_host:$remote_path/src/data_collection/"
rsync -a --protect-args --exclude='__pycache__/' --exclude='*.pyc' \
    "$CAMERA_DEPLOY_ROOT/src/data_collection_interfaces/" "$remote_host:$remote_path/src/data_collection_interfaces/"
rsync -a --protect-args "$CAMERA_DEPLOY_ROOT/requirements-recording.txt" "$remote_host:$remote_path/"
rsync -a --protect-args "$CAMERA_DEPLOY_ROOT/tools/setup_camera.sh" \
    "$CAMERA_DEPLOY_ROOT/tools/configure.py" "$CAMERA_DEPLOY_ROOT/tools/camera_env.sh" \
    "$remote_host:$remote_path/tools/"
rsync -a --protect-args "$CAMERA_DEPLOY_ROOT/docs/CAMERAS.md" \
    "$CAMERA_DEPLOY_ROOT/docs/RECORDING.md" "$remote_host:$remote_path/docs/"
printf -v setup_command '%q ' bash "$remote_path/tools/setup_camera.sh" "${setup_options[@]}"
if [[ " ${setup_options[*]} " != *" --user-deps "* && " ${setup_options[*]} " != *" --skip-system "* ]]; then
    if [[ ! -t 0 ]]; then
        printf 'System installation needs an interactive sudo terminal. Use --user-deps, or install on the Jetson first and use --skip-system.\n' >&2
        exit 2
    fi
    ssh -t "$remote_host" "$setup_command"
else
    ssh "$remote_host" "$setup_command"
fi
