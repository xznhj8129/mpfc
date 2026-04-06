#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
env_file=${HIVEOS_ENV_FILE:-"${repo_root}/config/pi/hiveos.env"}

source "${env_file}"

if [ "$#" -ne 1 ]; then
    echo "usage=$0 {start|stop|status}" >&2
    exit 1
fi

action=$1

case "${action}" in
    start)
        /usr/bin/docker rm -f "${HIVEOS_CONTAINER_NAME}" >/dev/null 2>&1 || true

        args=(
            run
            --rm
            --name "${HIVEOS_CONTAINER_NAME}"
            --network host
            --env-file "${env_file}"
            -v "${repo_root}:/opt/hiveos"
        )

        if [ -e /dev/bus/usb ]; then
            args+=(-v /dev/bus/usb:/dev/bus/usb)
        fi

        shopt -s nullglob
        for device in /dev/ttyUSB* /dev/ttyACM* /dev/serial0 /dev/serial1 /dev/gpiomem /dev/gpiochip* /dev/video* /dev/i2c-* /dev/spidev*; do
            args+=(--device "${device}")
        done
        shopt -u nullglob

        exec /usr/bin/docker "${args[@]}" "${HIVEOS_IMAGE}"
        ;;
    stop)
        /usr/bin/docker stop -t 10 "${HIVEOS_CONTAINER_NAME}" >/dev/null 2>&1 || true
        /usr/bin/docker rm -f "${HIVEOS_CONTAINER_NAME}" >/dev/null 2>&1 || true
        ;;
    status)
        exec /usr/bin/docker ps --filter "name=^/${HIVEOS_CONTAINER_NAME}$"
        ;;
    *)
        echo "invalid_action=${action}" >&2
        exit 1
        ;;
esac
