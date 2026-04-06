#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

chmod +x "${repo_root}/scripts/hiveos_docker_ctl.sh"

sudo install -m 0644 "${repo_root}/config/hiveos-docker.service" /etc/systemd/system/hiveos-docker.service
sudo systemctl daemon-reload
sudo systemctl enable --now hiveos-docker.service
