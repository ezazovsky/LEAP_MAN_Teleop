#!/usr/bin/env bash
# Launch the MANUS SDK client in a new terminal window and then run
# combined_simple_teleop_real_logger.py in the current terminal.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SDK_BIN="$REPO_ROOT/LMAPI/MANUS_Core_2.4.0_SDK/SDKClient_Linux/SDKClient_Linux.out"
TELEOP_PY="$SCRIPT_DIR/combined_simple_teleop_real_logger.py"

# Choose virtualenv: prefer VENV env var, then .venv39, then .venv
VENV="${VENV:-$REPO_ROOT/.venv39}"
if [ ! -x "$VENV/bin/python" ]; then
    VENV="$REPO_ROOT/.venv"
fi

if [ ! -x "$VENV/bin/python" ]; then
    echo "Warning: no virtualenv found at .venv39 or .venv. Teleop will use system python."
    PY_CMD="python3"
else
    PY_CMD="$VENV/bin/python"
fi

SDK_CMD="\"$SDK_BIN\""

# Allow overriding HOME for SteamVR compatibility (see docs)
HOME_PREFIX=""
if [ -n "${STEAMVR_HOME-}" ]; then
    HOME_PREFIX="HOME=$STEAMVR_HOME "
fi

# Build the command to run the SDK
SDK_RUN_CMD="$HOME_PREFIX$SDK_BIN"

# Find a terminal emulator to open
TERMS=(gnome-terminal konsole xfce4-terminal xterm alacritty tilix mate-terminal)
LAUNCHED_SDK=""
for term in "${TERMS[@]}"; do
    if command -v "$term" >/dev/null 2>&1; then
        case "$term" in
            gnome-terminal)
                "$term" -- bash -c "$SDK_RUN_CMD; echo 'MANUS SDK exited'; exec bash" &
                LAUNCHED_SDK=1
                break
                ;;
            konsole)
                "$term" -e bash -c "$SDK_RUN_CMD; echo 'MANUS SDK exited'; exec bash" &
                LAUNCHED_SDK=1
                break
                ;;
            xfce4-terminal)
                "$term" --command="bash -c '$SDK_RUN_CMD; echo MANUS SDK exited; exec bash'" &
                LAUNCHED_SDK=1
                break
                ;;
            tilix)
                "$term" -e bash -c "$SDK_RUN_CMD; echo 'MANUS SDK exited'; exec bash" &
                LAUNCHED_SDK=1
                break
                ;;
            mate-terminal)
                "$term" -- bash -c "$SDK_RUN_CMD; echo 'MANUS SDK exited'; exec bash" &
                LAUNCHED_SDK=1
                break
                ;;
            alacritty)
                "$term" -e bash -c "$SDK_RUN_CMD; echo 'MANUS SDK exited'; read -n1 -r -p 'Press any key to close...'" &
                LAUNCHED_SDK=1
                break
                ;;
            xterm)
                "$term" -hold -e bash -c "$SDK_RUN_CMD" &
                LAUNCHED_SDK=1
                break
                ;;
        esac
    fi
done

if [ -z "$LAUNCHED_SDK" ]; then
    echo "No graphical terminal emulator found — launching MANUS SDK in background (logs: Combined/manus_sdk.log)"
    mkdir -p "$SCRIPT_DIR/logs"
    nohup $SDK_RUN_CMD > "$SCRIPT_DIR/logs/manus_sdk.log" 2>&1 &
fi

echo "Started MANUS SDK (if available). Now launching teleop logger in this terminal using: $PY_CMD"

# Finally run the teleop script in this terminal
exec $PY_CMD "$TELEOP_PY" "$@"
