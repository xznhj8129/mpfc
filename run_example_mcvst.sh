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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --main-config)
            MCVST_MAIN_CONFIG="$2"
            shift 2
            ;;
        --my-name)
            MY_NAME="$2"
            shift 2
            ;;
        *)
            echo "[RUN] unknown arg arg=$1" >&2
            exit 1
            ;;
    esac
done

MCVST_MAIN_CONFIG="${MCVST_MAIN_CONFIG:-flight_cores/example_mcvst/config.yaml}"
MY_NAME="${MY_NAME:-cv1}"

main_args=(--my_name="$MY_NAME")
MAIN_CONFIG="$MCVST_MAIN_CONFIG"
export MAIN_CONFIG
echo "[RUN] MAIN_CONFIG=$MAIN_CONFIG my_name=$MY_NAME" >&2
env -u LD_LIBRARY_PATH python main.py "${main_args[@]}"
