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

# Hive run settings.
MAV_MAIN_CONFIG="${MAV_MAIN_CONFIG:-flight_cores/test_takeoff_land/config_px4.yaml}"
MY_NAME="${MY_NAME:-uav1}"
MAV_PORT="${MAV_PORT:-14550}"

# PX4/Gazebo settings.
START_PX4_SITL="${START_PX4_SITL:-0}"
PX4_DIR="${PX4_DIR:-/media/anon/WD2TB/DataVault/TechProjects/Software/other_software/PX4-Autopilot}"
PX4_VEHICLE_TARGET="${PX4_VEHICLE_TARGET:-gz_x500_mono_cam}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-}"
PX4_HOME_LAT="${PX4_HOME_LAT:-}"
PX4_HOME_LON="${PX4_HOME_LON:-}"
PX4_HOME_ALT="${PX4_HOME_ALT:-}"
PX4_GZ_IP="${PX4_GZ_IP:-127.0.0.1}"
PX4_GZ_GUI="${PX4_GZ_GUI:-1}"
PX4_VIDEO_HOST_IP="${PX4_VIDEO_HOST_IP:-127.0.0.1}"
PX4_VIDEO_UDP_PORT="${PX4_VIDEO_UDP_PORT:-5601}"
QGC_MAVLINK_PORT="${QGC_MAVLINK_PORT:-14551}"
PX4_ENABLE_QGC_MAVLINK="${PX4_ENABLE_QGC_MAVLINK:-1}"
PX4_QGC_LOCAL_PORT="${PX4_QGC_LOCAL_PORT:-18571}"
PX4_LOG_FILE="${PX4_LOG_FILE:-$SCRIPT_DIR/px4_sitl_${MY_NAME}_${MAV_PORT}.log}"
PX4_SHELL_PID_FILE="${PX4_SHELL_PID_FILE:-$SCRIPT_DIR/.px4_shell_${MY_NAME}_${MAV_PORT}.pid}"

PX4_TERM_PID=""
PX4_AUTOSTART_POST_FILE=""
PX4_AUTOSTART_POST_BACKUP=""

if [[ "${LD_LIBRARY_PATH:-}" == *"/tmp/.mount_vscodi"* ]]; then
    unset LD_LIBRARY_PATH
fi

cleanup() {
    if [[ -f "$PX4_SHELL_PID_FILE" ]]; then
        px4_shell_pid="$(cat "$PX4_SHELL_PID_FILE" 2>/dev/null || true)"
        if [[ -n "${px4_shell_pid:-}" ]]; then
            kill -TERM -- "-$px4_shell_pid" 2>/dev/null || true
            sleep 0.5
            kill -KILL -- "-$px4_shell_pid" 2>/dev/null || true
        fi
        rm -f "$PX4_SHELL_PID_FILE"
    fi
    if [[ -n "$PX4_AUTOSTART_POST_FILE" ]]; then
        if [[ -n "$PX4_AUTOSTART_POST_BACKUP" && -f "$PX4_AUTOSTART_POST_BACKUP" ]]; then
            mv "$PX4_AUTOSTART_POST_BACKUP" "$PX4_AUTOSTART_POST_FILE"
        else
            rm -f "$PX4_AUTOSTART_POST_FILE"
        fi
    fi
    if [[ -n "$PX4_TERM_PID" ]]; then
        kill "$PX4_TERM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [[ "$START_PX4_SITL" == "1" ]]; then
    world_file="$PX4_DIR/Tools/simulation/gz/worlds/$PX4_GZ_WORLD.sdf"
    if [[ ! -f "$world_file" ]]; then
        echo "[RUN] world not found world=$PX4_GZ_WORLD expected=$world_file" >&2
        exit 1
    fi

    if [[ "$PX4_ENABLE_QGC_MAVLINK" == "1" ]]; then
        autostart_dir="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
        if [[ ! -d "$autostart_dir" ]]; then
            autostart_dir="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
        fi
        autostart_file="$(ls "$autostart_dir"/*_"$PX4_VEHICLE_TARGET" 2>/dev/null | head -n 1)"
        if [[ -z "$autostart_file" ]]; then
            echo "[RUN] autostart file not found vehicle=$PX4_VEHICLE_TARGET dir=$autostart_dir" >&2
            exit 1
        fi

        PX4_AUTOSTART_POST_FILE="${autostart_file}.post"
        if [[ -f "$PX4_AUTOSTART_POST_FILE" ]]; then
            PX4_AUTOSTART_POST_BACKUP="${PX4_AUTOSTART_POST_FILE}.hiveos.bak"
            cp "$PX4_AUTOSTART_POST_FILE" "$PX4_AUTOSTART_POST_BACKUP"
        fi

        {
            if [[ -n "$PX4_AUTOSTART_POST_BACKUP" ]]; then
                cat "$PX4_AUTOSTART_POST_BACKUP"
            fi
            echo "# HiveOS: dedicated QGC MAVLink output"
            echo "mavlink start -x -u $PX4_QGC_LOCAL_PORT -r 4000000 -f -o $QGC_MAVLINK_PORT"
            echo "mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $PX4_QGC_LOCAL_PORT"
            echo "mavlink stream -r 50 -s ATTITUDE -u $PX4_QGC_LOCAL_PORT"
            echo "mavlink stream -r 20 -s RC_CHANNELS -u $PX4_QGC_LOCAL_PORT"
            echo "mavlink stream -r 10 -s SYS_STATUS -u $PX4_QGC_LOCAL_PORT"
        } >"$PX4_AUTOSTART_POST_FILE"
        echo "[RUN] QGC mavlink post file path=$PX4_AUTOSTART_POST_FILE local_port=$PX4_QGC_LOCAL_PORT remote_port=$QGC_MAVLINK_PORT" >&2
    fi

    rm -f "$PX4_SHELL_PID_FILE"
    PX4_CMD="cd '$PX4_DIR'; echo \$\$ > '$PX4_SHELL_PID_FILE'; export PX4_GZ_WORLD='$PX4_GZ_WORLD'; export PX4_GZ_MODEL_POSE='$PX4_GZ_MODEL_POSE'; export PX4_HOME_LAT='$PX4_HOME_LAT'; export PX4_HOME_LON='$PX4_HOME_LON'; export PX4_HOME_ALT='$PX4_HOME_ALT'; export GZ_IP='$PX4_GZ_IP'; export PX4_GZ_GUI='$PX4_GZ_GUI'; export PX4_VIDEO_HOST_IP='$PX4_VIDEO_HOST_IP'; export PX4_VIDEO_UDP_PORT='$PX4_VIDEO_UDP_PORT'; set -o pipefail; env -u LD_LIBRARY_PATH make px4_sitl '$PX4_VEHICLE_TARGET' 2>&1 | tee '$PX4_LOG_FILE'; exit_code=\$?; echo \"[RUN] PX4 terminal exited status=\$exit_code\""
    gnome-terminal --title="PX4 SITL $MY_NAME" -- bash -lc "$PX4_CMD" &
    PX4_TERM_PID="$!"
    echo "[RUN] started PX4 SITL pid=$PX4_TERM_PID vehicle=$PX4_VEHICLE_TARGET world=$PX4_GZ_WORLD qgc_mavlink=udp://$QGC_MAVLINK_PORT qgc_video=${PX4_VIDEO_HOST_IP}:${PX4_VIDEO_UDP_PORT} log=$PX4_LOG_FILE qgc_extra_link=$PX4_ENABLE_QGC_MAVLINK" >&2
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
