#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec python -m field_launcher --config "${1:?Prepared field YAML is required}" share
