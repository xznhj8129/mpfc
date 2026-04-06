#!/usr/bin/env bash
set -euo pipefail

declare -a child_pids=()
declare -A child_names=()

start_child() {
    local name=$1
    shift
    "$@" &
    local pid=$!
    child_pids+=("$pid")
    child_names["$pid"]="$name"
    echo "[ENTRYPOINT] name=$name pid=$pid argv=$*" >&2
}

stop_children() {
    if [ ${#child_pids[@]} -eq 0 ]; then
        return
    fi
    echo "[ENTRYPOINT] stop_pids=${child_pids[*]} main_config=${MAIN_CONFIG}" >&2
    kill -TERM "${child_pids[@]}" 2>/dev/null || true
    wait "${child_pids[@]}" 2>/dev/null || true
}

on_signal() {
    local signal=$1
    echo "[ENTRYPOINT] signal=${signal} main_config=${MAIN_CONFIG}" >&2
    stop_children
    exit 0
}

trap 'on_signal SIGINT' SIGINT
trap 'on_signal SIGTERM' SIGTERM

cd "${HIVEOS_WORKDIR}"

echo "[ENTRYPOINT] start_mosquitto=${HIVEOS_START_MOSQUITTO} start_mavlink_router=${HIVEOS_START_MAVLINK_ROUTER} main_config=${MAIN_CONFIG} workdir=${HIVEOS_WORKDIR}" >&2

if [ "${HIVEOS_START_MOSQUITTO}" = "1" ]; then
    mkdir -p /run/mosquitto /var/log/mosquitto
    chown mosquitto:mosquitto /run/mosquitto /var/log/mosquitto
    start_child mosquitto mosquitto -c "${MOSQUITTO_CONFIG}"
fi

if [ "${HIVEOS_START_MAVLINK_ROUTER}" = "1" ]; then
    start_child mavlink-routerd mavlink-routerd -c "${MAVLINK_ROUTER_CONFIG}"
fi

start_child hiveos python -u main.py "$@"

wait -n -p exited_pid "${child_pids[@]}"
status=$?
echo "[ENTRYPOINT] exited_name=${child_names[$exited_pid]} exited_pid=${exited_pid} status=${status} main_config=${MAIN_CONFIG}" >&2
stop_children
exit "${status}"
