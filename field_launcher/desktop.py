"""Prepared freedesktop launcher pins the installed interpreter, checkout and device configuration."""
import sys
from dataclasses import dataclass
from pathlib import Path


def desktop_argument(value) -> str:
    escaped = str(value).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$')
    return '"' + escaped.replace('%', '%%') + '"'


@dataclass(frozen=True)
class DesktopLauncher:
    config: Path
    directory: Path

    def write(self, output: Path):
        command = ' '.join(desktop_argument(value) for value in
            (sys.executable, "-m", "field_launcher", "--config", self.config.resolve(), "share"))
        output.write_text("[Desktop Entry]\nType=Application\nName=Share from this laptop\n"
            f"Exec={command}\nPath={self.directory}\nIcon=camera-video\nTerminal=false\n")
        output.chmod(0o700)
