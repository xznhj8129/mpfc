#!/usr/bin/env python3
"""
Usage:
    from plugins.px4_sitl_gz_launcher.px4_sitl_gz_launcher import run_plugin
    run_plugin(cfg, bus_config)
"""

import os
import shlex
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict

from lib.plugin_base import PluginBase


class Px4SitlGzLauncher(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        self.auto_start = bool(cfg["auto_start"])
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.px4_cfg = cfg["px4"]
        self.gz_cfg = cfg["gazebo"]
        self.px4_dir = Path(self.px4_cfg["px4_dir"]).resolve()
        self.log_file = Path(cfg["log_file"]).resolve()
        self.shell_pid_file = Path(cfg["shell_pid_file"]).resolve()
        self.term_proc: subprocess.Popen | None = None
        self.autostart_post_file: Path | None = None
        self.autostart_post_backup: Path | None = None

    def _write_qgc_post_file(self) -> None:
        if not bool(self.px4_cfg["qgc_mavlink_enabled"]):
            return

        sitl_vehicle = self.px4_cfg["sitl_vehicle"]
        autostart_dir = self.px4_dir / "build/px4_sitl_default/etc/init.d-posix/airframes"
        if not autostart_dir.is_dir():
            autostart_dir = self.px4_dir / "ROMFS/px4fmu_common/init.d-posix/airframes"
        autostart_file = sorted(autostart_dir.glob(f"*_{sitl_vehicle}"))[0]

        self.autostart_post_file = autostart_file.with_name(f"{autostart_file.name}.post")
        if self.autostart_post_file.exists():
            self.autostart_post_backup = self.autostart_post_file.with_name(f"{self.autostart_post_file.name}.hiveos.bak")
            self.autostart_post_backup.write_bytes(self.autostart_post_file.read_bytes())

        qgc_local_port = self.px4_cfg["qgc_local_port"]
        qgc_mavlink_port = self.px4_cfg["qgc_mavlink_port"]
        lines = []
        if self.autostart_post_backup is not None:
            lines.extend(self.autostart_post_backup.read_text(encoding="utf-8").splitlines())
        lines.extend(
            [
                "# HiveOS: dedicated QGC MAVLink output",
                f"mavlink start -x -u {qgc_local_port} -r 4000000 -f -o {qgc_mavlink_port}",
                f"mavlink stream -r 50 -s GLOBAL_POSITION_INT -u {qgc_local_port}",
                f"mavlink stream -r 50 -s ATTITUDE -u {qgc_local_port}",
                f"mavlink stream -r 20 -s RC_CHANNELS -u {qgc_local_port}",
                f"mavlink stream -r 10 -s SYS_STATUS -u {qgc_local_port}",
            ]
        )
        self.autostart_post_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(
            f"[PX4_SITL] id={self.client_id} qgc_post_file={self.autostart_post_file} "
            f"local_port={qgc_local_port} remote_port={qgc_mavlink_port}",
            flush=True,
        )

    def _restore_qgc_post_file(self) -> None:
        if self.autostart_post_file is None:
            return
        if self.autostart_post_backup is not None and self.autostart_post_backup.exists():
            self.autostart_post_file.write_bytes(self.autostart_post_backup.read_bytes())
            self.autostart_post_backup.unlink()
            return
        self.autostart_post_file.unlink(missing_ok=True)

    def _start_px4(self) -> None:
        if not self.auto_start:
            return
        if self.term_proc is not None and self.term_proc.poll() is None:
            return

        sitl_vehicle = self.px4_cfg["sitl_vehicle"]
        gz_world = self.gz_cfg["gz_world"]
        world_file = self.px4_dir / "Tools/simulation/gz/worlds" / f"{gz_world}.sdf"
        if not world_file.is_file():
            raise RuntimeError(f"world not found world={gz_world} expected={world_file}")

        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self.shell_pid_file.unlink(missing_ok=True)
        self._write_qgc_post_file()

        env_values = {
            "PX4_GZ_WORLD": gz_world,
            "PX4_GZ_MODEL_POSE": self.px4_cfg["model_pose"],
            "PX4_HOME_LAT": self.px4_cfg["home_lat"],
            "PX4_HOME_LON": self.px4_cfg["home_lon"],
            "PX4_HOME_ALT": self.px4_cfg["home_alt"],
            "GZ_IP": self.gz_cfg["gz_ip"],
            "PX4_GZ_GUI": 1 if bool(self.gz_cfg["gz_gui"]) else 0,
            "PX4_VIDEO_HOST_IP": self.gz_cfg["video_host_ip"],
            "PX4_VIDEO_UDP_PORT": self.gz_cfg["video_host_port"],
        }
        exports = "; ".join(f"export {key}={shlex.quote(str(value))}" for key, value in env_values.items())
        cmd = (
            f"cd {shlex.quote(str(self.px4_dir))}; "
            f"echo $$ > {shlex.quote(str(self.shell_pid_file))}; "
            f"{exports}; "
            f"set -o pipefail; "
            f"env -u LD_LIBRARY_PATH make px4_sitl {shlex.quote(str(sitl_vehicle))} 2>&1 | tee {shlex.quote(str(self.log_file))}; "
            f"exit_code=$?; "
            f"echo \"[PX4_SITL] exited status=$exit_code\"; "
            f"exit $exit_code"
        )
        self.term_proc = subprocess.Popen(
            ["gnome-terminal", f"--title=PX4 SITL {self.client_id}", "--", "bash", "-lc", cmd]
        )
        print(
            f"[PX4_SITL] id={self.client_id} pid={self.term_proc.pid} vehicle={sitl_vehicle} world={gz_world} "
            f"companion_mavlink=udp://:{self.px4_cfg['companion_mavlink_port']} "
            f"qgc_mavlink=udp://:{self.px4_cfg['qgc_mavlink_port']} "
            f"video={self.gz_cfg['video_host_ip']}:{self.gz_cfg['video_host_port']} log={self.log_file}",
            flush=True,
        )

    def _stop_px4(self) -> None:
        if self.shell_pid_file.exists():
            shell_pid = int(self.shell_pid_file.read_text(encoding="utf-8").strip())
            try:
                os.killpg(shell_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(0.5)
            try:
                os.killpg(shell_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.shell_pid_file.unlink(missing_ok=True)
        if self.term_proc is not None and self.term_proc.poll() is None:
            self.term_proc.terminate()
            try:
                self.term_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.term_proc.kill()
                self.term_proc.wait()
        self._restore_qgc_post_file()

    def run(self) -> None:
        self.send_online()
        try:
            self._start_px4()
            while True:
                self.recv_until(time.monotonic() + self.poll_interval_s)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_px4()
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    Px4SitlGzLauncher(cfg, bus_config).run()
