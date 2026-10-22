#!/usr/bin/env bash
# Explicit board initialization; no image capture is started.
set -euo pipefail
if [[ "$EUID" -ne 0 ]]; then
    exec sudo --preserve-env=GMSL_VENDOR_DIR bash "$0" "$@"
fi
GMSL_VENDOR_DIR="${GMSL_VENDOR_DIR:-/home/nvidia/cam_geac}"
[[ -x "$GMSL_VENDOR_DIR/rb_camera.sh" ]] || {
    printf 'Install the Tianji GEAC BSP and set GMSL_VENDOR_DIR to its directory.\n' >&2; exit 1;
}
read -r boot_id < /proc/sys/kernel/random/boot_id
marker="/tmp/tj_cam_geac_init_$boot_id"
exec 9>/run/lock/tj-camera-init.lock
flock 9
if [[ -f "$marker" ]]; then
    [[ "$(cat "$marker")" == completed ]] || {
        printf 'GEAC initialization did not complete in this boot; reboot before initializing.\n' >&2; exit 1;
    }
    printf 'GEAC cameras are already initialized for this boot.\n'
    exit 0
fi
printf 'started\n' > "$marker"
cd "$GMSL_VENDOR_DIR"
./rb_camera.sh init 15
for channel in 0 1 4 5; do
    [[ -e "/dev/video$channel" ]] || { printf 'Missing /dev/video%s\n' "$channel" >&2; exit 1; }
done
printf 'completed\n' > "$marker"
printf 'GEAC initialization complete.\n'
