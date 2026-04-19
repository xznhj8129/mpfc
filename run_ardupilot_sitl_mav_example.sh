#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-$SCRIPT_DIR/.env}"
if [[ -f "$ENV_FILE" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$ENV_FILE"
    set +a
fi

cd "$SCRIPT_DIR"

# Hive run settings.
MAV_MAIN_CONFIG="${MAV_MAIN_CONFIG:-flight_cores/test_takeoff_land/config_ardupilot.yaml}"
MY_NAME="${MY_NAME:-uav1}"
MAV_PORT="${MAV_PORT:-14550}"

# ArduPilot SITL settings.
START_ARDUPILOT_SITL="${START_ARDUPILOT_SITL:-1}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$SCRIPT_DIR/../../../other_software/ardupilot}"
ARDUPILOT_VEHICLE="${ARDUPILOT_VEHICLE:-ArduCopter}"
ARDUPILOT_FRAME="${ARDUPILOT_FRAME:-X}"
ARDUPILOT_HOME_LAT="${ARDUPILOT_HOME_LAT:-42.30}"
ARDUPILOT_HOME_LON="${ARDUPILOT_HOME_LON:--71.9}"
ARDUPILOT_HOME_ALT="${ARDUPILOT_HOME_ALT:-0}"
ARDUPILOT_HOME_HEADING="${ARDUPILOT_HOME_HEADING:-0}"
ARDUPILOT_SPEEDUP="${ARDUPILOT_SPEEDUP:-1}"
ARDUPILOT_INSTANCE="${ARDUPILOT_INSTANCE:-0}"
ARDUPILOT_SYSID="${ARDUPILOT_SYSID:-1}"
ARDUPILOT_USE_MAP="${ARDUPILOT_USE_MAP:-1}"
ARDUPILOT_USE_CONSOLE="${ARDUPILOT_USE_CONSOLE:-1}"
ARDUPILOT_NO_REBUILD="${ARDUPILOT_NO_REBUILD:-1}"
ARDUPILOT_WIPE_EEPROM="${ARDUPILOT_WIPE_EEPROM:-0}"
ARDUPILOT_MAVPROXY_ARGS="${ARDUPILOT_MAVPROXY_ARGS:-}"
ARDUPILOT_SITL_INSTANCE_ARGS="${ARDUPILOT_SITL_INSTANCE_ARGS:-}"
QGC_MAVLINK_PORT="${QGC_MAVLINK_PORT:-14551}"
MQTT_LOG="${MQTT_LOG:-1}"
MQTT_LOG_FILE="${MQTT_LOG_FILE:-$SCRIPT_DIR/logs/mqtt.log}"
ARDUPILOT_LOG_FILE="${ARDUPILOT_LOG_FILE:-$SCRIPT_DIR/logs/ardupilot_sitl_${MY_NAME}_${MAV_PORT}.log}"
ARDUPILOT_USE_DIR="${ARDUPILOT_USE_DIR:-$SCRIPT_DIR/logs/ardupilot_sitl_${MY_NAME}_${MAV_PORT}}"
ARDUPILOT_SHELL_PID_FILE="${ARDUPILOT_SHELL_PID_FILE:-$SCRIPT_DIR/.ardupilot_shell_${MY_NAME}_${MAV_PORT}.pid}"
MQTT_LOG_PID_FILE="${MQTT_LOG_PID_FILE:-$SCRIPT_DIR/.mqtt_log_${MY_NAME}_${MAV_PORT}.pid}"

ARDUPILOT_TERM_PID=""

if [[ "${LD_LIBRARY_PATH:-}" == *"/tmp/.mount_vscodi"* ]]; then
    unset LD_LIBRARY_PATH
fi

cleanup() {
    if [[ -f "$MQTT_LOG_PID_FILE" ]]; then
        mqtt_log_pid="$(cat "$MQTT_LOG_PID_FILE" 2>/dev/null || true)"
        if [[ -n "${mqtt_log_pid:-}" ]]; then
            kill -TERM -- "-$mqtt_log_pid" 2>/dev/null || true
            sleep 0.5
            kill -KILL -- "-$mqtt_log_pid" 2>/dev/null || true
        fi
        rm -f "$MQTT_LOG_PID_FILE"
    fi
    if [[ -f "$ARDUPILOT_SHELL_PID_FILE" ]]; then
        ardupilot_shell_pid="$(cat "$ARDUPILOT_SHELL_PID_FILE" 2>/dev/null || true)"
        if [[ -n "${ardupilot_shell_pid:-}" ]]; then
            kill -TERM -- "-$ardupilot_shell_pid" 2>/dev/null || true
            sleep 0.5
            kill -KILL -- "-$ardupilot_shell_pid" 2>/dev/null || true
        fi
        rm -f "$ARDUPILOT_SHELL_PID_FILE"
    fi
    if [[ -n "$ARDUPILOT_TERM_PID" ]]; then
        kill "$ARDUPILOT_TERM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [[ "$MQTT_LOG" == "1" ]]; then
    mkdir -p "$(dirname "$MQTT_LOG_FILE")"
    rm -f "$MQTT_LOG_PID_FILE"
    MQTT_LOG_CMD="echo \$\$ > '$MQTT_LOG_PID_FILE'; exec mosquitto_sub -h 127.0.0.1 -t '#' -v > '$MQTT_LOG_FILE'"
    setsid bash -lc "$MQTT_LOG_CMD" >/dev/null 2>&1 &
    echo "[RUN] started MQTT log pid=$! file=$MQTT_LOG_FILE" >&2
fi

if [[ "$START_ARDUPILOT_SITL" == "1" ]]; then
    if [[ ! -d "$ARDUPILOT_DIR" ]]; then
        echo "[RUN] ardupilot dir not found path=$ARDUPILOT_DIR" >&2
        exit 1
    fi

    ARDUPILOT_DIR="$(cd "$ARDUPILOT_DIR" && pwd)"
    mkdir -p "$(dirname "$ARDUPILOT_LOG_FILE")"
    mkdir -p "$ARDUPILOT_USE_DIR"
    rm -f "$ARDUPILOT_SHELL_PID_FILE"

    ardupilot_location="${ARDUPILOT_HOME_LAT},${ARDUPILOT_HOME_LON},${ARDUPILOT_HOME_ALT},${ARDUPILOT_HOME_HEADING}"
    sim_vehicle_args=(
        python
        Tools/autotest/sim_vehicle.py
        -v "$ARDUPILOT_VEHICLE"
        -f "$ARDUPILOT_FRAME"
        -I "$ARDUPILOT_INSTANCE"
        --sysid "$ARDUPILOT_SYSID"
        -S "$ARDUPILOT_SPEEDUP"
        -l "$ardupilot_location"
        --use-dir "$ARDUPILOT_USE_DIR"
        --out="udp:127.0.0.1:$MAV_PORT"
        --no-extra-ports
    )
    if [[ -n "$QGC_MAVLINK_PORT" && "$QGC_MAVLINK_PORT" != "$MAV_PORT" ]]; then
        sim_vehicle_args+=(--out="udp:127.0.0.1:$QGC_MAVLINK_PORT")
    fi
    if [[ "$ARDUPILOT_USE_MAP" == "1" ]]; then
        sim_vehicle_args+=(--map)
    fi
    if [[ "$ARDUPILOT_USE_CONSOLE" == "1" ]]; then
        sim_vehicle_args+=(--console)
    fi
    if [[ "$ARDUPILOT_NO_REBUILD" == "1" ]]; then
        sim_vehicle_args+=(-N)
    fi
    if [[ "$ARDUPILOT_WIPE_EEPROM" == "1" ]]; then
        sim_vehicle_args+=(-w)
    fi
    if [[ -n "$ARDUPILOT_MAVPROXY_ARGS" ]]; then
        sim_vehicle_args+=(-m "$ARDUPILOT_MAVPROXY_ARGS")
    fi
    if [[ -n "$ARDUPILOT_SITL_INSTANCE_ARGS" ]]; then
        sim_vehicle_args+=(-A "$ARDUPILOT_SITL_INSTANCE_ARGS")
    fi

    printf -v sim_vehicle_cmd '%q ' "${sim_vehicle_args[@]}"
    ARDUPILOT_CMD="cd '$ARDUPILOT_DIR'; echo \$\$ > '$ARDUPILOT_SHELL_PID_FILE'; set -o pipefail; env -u LD_LIBRARY_PATH ${sim_vehicle_cmd}2>&1 | tee '$ARDUPILOT_LOG_FILE'; exit_code=\$?; echo \"[RUN] ArduPilot terminal exited status=\$exit_code\""
    ARDUPILOT_HEADLESS_CMD="cd '$ARDUPILOT_DIR'; echo \$\$ > '$ARDUPILOT_SHELL_PID_FILE'; set -o pipefail; tail -f /dev/null | env -u LD_LIBRARY_PATH ${sim_vehicle_cmd}2>&1 | tee '$ARDUPILOT_LOG_FILE'; exit_code=\$?; echo \"[RUN] ArduPilot terminal exited status=\$exit_code\""
    if [[ -n "${DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="ArduPilot SITL $MY_NAME" -- bash -lc "$ARDUPILOT_CMD" &
        ARDUPILOT_TERM_PID="$!"
        echo "[RUN] started ArduPilot SITL terminal_pid=$ARDUPILOT_TERM_PID vehicle=$ARDUPILOT_VEHICLE frame=$ARDUPILOT_FRAME location=$ardupilot_location mav_port=$MAV_PORT qgc_port=$QGC_MAVLINK_PORT use_dir=$ARDUPILOT_USE_DIR log=$ARDUPILOT_LOG_FILE" >&2
    else
        setsid bash -lc "$ARDUPILOT_HEADLESS_CMD" >/dev/null 2>&1 &
        ARDUPILOT_TERM_PID="$!"
        echo "[RUN] started ArduPilot SITL headless_pid=$ARDUPILOT_TERM_PID vehicle=$ARDUPILOT_VEHICLE frame=$ARDUPILOT_FRAME location=$ardupilot_location mav_port=$MAV_PORT qgc_port=$QGC_MAVLINK_PORT use_dir=$ARDUPILOT_USE_DIR log=$ARDUPILOT_LOG_FILE" >&2
    fi
else
    echo "[RUN] ArduPilot SITL disabled START_ARDUPILOT_SITL=$START_ARDUPILOT_SITL mav_port=$MAV_PORT" >&2
fi

core_cfg_has_key() {
    local key="$1"
    python - "$MAV_MAIN_CONFIG" "$key" <<'PY'
import sys
import yaml

cfg_path = sys.argv[1]
key = sys.argv[2]
with open(cfg_path, "r", encoding="utf-8") as handle:
    cfg = yaml.safe_load(handle)
print("1" if key in cfg["core"]["cfg"] else "0")
PY
}

main_args=(--my_name="$MY_NAME")
if [[ -n "$MAV_PORT" && "$(core_cfg_has_key mav_port)" == "1" ]]; then
    main_args+=(--mav_port="$MAV_PORT")
fi

MAIN_CONFIG="$MAV_MAIN_CONFIG"
export MAIN_CONFIG
echo "[RUN] MAIN_CONFIG=$MAIN_CONFIG my_name=$MY_NAME mav_port=$MAV_PORT" >&2
env -u LD_LIBRARY_PATH python main.py "${main_args[@]}"
