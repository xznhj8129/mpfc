"""Owned process groups; MPFC remains the telemetry supervisor, FFmpeg the encoder."""
import os
import signal
import subprocess
import time
from pathlib import Path


class ManagedProcess:
    def __init__(self, command: list[str], directory: Path, log: Path, environment=None):
        self.command, self.directory, self.log, self.environment = command, directory, log, environment
        self.process: subprocess.Popen | None = None
        self.output = None
        self.next_start = 0.0

    def start(self):
        self.output = self.log.open("ab")
        self.process = subprocess.Popen(self.command, cwd=self.directory, env=self.environment,
                                        stdout=self.output, stderr=subprocess.STDOUT, start_new_session=True)

    def maintain(self) -> bool:
        if self.process is None:
            raise RuntimeError(f"Process has not started; log={self.log}")
        if self.process.poll() is None:
            return True
        # A supervisor can exit before its children; reap the entire owned group before retrying.
        self.stop()
        if time.monotonic() >= self.next_start:
            self.start()
            self.next_start = time.monotonic() + 3
        return False

    def stop(self):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass  # The complete process group already exited.
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3)
        if self.output is not None:
            self.output.close()
