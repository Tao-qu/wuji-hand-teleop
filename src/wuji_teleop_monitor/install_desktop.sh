#!/usr/bin/env bash
# Install local desktop shortcuts after building the workspace.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"

if [ ! -d "$DESKTOP_DIR" ]; then
    echo "[ERROR] Desktop directory not found: $DESKTOP_DIR" >&2
    exit 1
fi

PACKAGE_PATH="$SCRIPT_DIR/wuji_teleop_monitor"
LAUNCHER="$SCRIPT_DIR/scripts/launcher.sh"

if [ ! -f "$LAUNCHER" ]; then
    echo "[ERROR] Launcher script not found: $LAUNCHER" >&2
    exit 1
fi
if [ ! -f "$PACKAGE_PATH/wuji.svg" ]; then
    echo "[WARN] Icon not found: $PACKAGE_PATH/wuji.svg (.desktop will fall back to a generic icon)" >&2
fi

chmod +x "$LAUNCHER"

install_shortcut() {
    local ui_name="$1"
    local template_name="$2"
    local output_name="$3"
    local template="$SCRIPT_DIR/$template_name"
    local output="$DESKTOP_DIR/$output_name"

    if [ ! -f "$template" ]; then
        echo "[ERROR] Template not found: $template" >&2
        return 1
    fi
    sed -e "s|{{HOME}}|$HOME|g" \
        -e "s|{{PACKAGE_PATH}}|$PACKAGE_PATH|g" \
        -e "s|{{LAUNCHER}}|$LAUNCHER $ui_name|g" \
        "$template" > "$output"
    chmod +x "$output"
    gio set "$output" metadata::trusted true 2>/dev/null || true
    echo "[OK] $output"
}

install_shortcut monitor teleop-monitor.desktop.template teleop-monitor.desktop
install_shortcut brake   brake-control.desktop.template  brake-control.desktop

echo "Monitor and brake shortcuts installed in $DESKTOP_DIR"
