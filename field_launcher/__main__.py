"""Laptop entry point. Device preparation is separate from selecting sources and sharing."""
import argparse
import html
import logging
import signal
import subprocess
import threading
from pathlib import Path

import httpx
from anduril.core.api_error import ApiError

from field_launcher.config import FieldSettings
from field_launcher.desktop import DesktopLauncher
from field_launcher.registry import FieldRegistry
from field_launcher.session import FieldSession, ROOT

logger = logging.getLogger(__name__)


class LaptopWindow:
    def __init__(self):
        self.process = None

    def select(self, settings: FieldSettings):
        if not settings.assets:
            raise ValueError("This setup contains no laptop sources")
        rows = [value for asset in settings.assets for value in ("TRUE", asset.entity_id, asset.name)]
        result = subprocess.run(["zenity", "--list", "--checklist", "--title=Share from this laptop",
            "--text=Select prepared sources", "--column=Share", "--column=ID", "--column=Source",
            "--hide-column=2", "--print-column=2", "--separator=\n", "--ok-label=Share",
            "--width=520", "--height=320", *rows], text=True, capture_output=True)
        if result.returncode == 1:
            return []
        result.check_returncode()
        ids = set(result.stdout.splitlines())
        return [asset for asset in settings.assets if asset.entity_id in ids]

    def open(self):
        self.process = subprocess.Popen(["zenity", "--progress", "--pulsate", "--no-markup",
            "--title=Field sharing", "--text=Connecting…", "--cancel-label=Stop",
            "--width=650"], stdin=subprocess.PIPE, text=True)

    def update(self, message: str) -> bool:
        if self.process.poll() is not None:
            return False
        try:
            self.process.stdin.write("# " + message.replace("\\", "\\\\").replace("\n", "\\n") + "\n")
            self.process.stdin.flush()
        except BrokenPipeError:
            return False  # The operator closed the progress window.
        return True

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
            self.process.wait(timeout=3)


def share(settings: FieldSettings, headless: bool):
    window = None if headless else LaptopWindow()
    selected = settings.assets if headless else window.select(settings)
    if not selected:
        return
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    previous = None
    try:
        if window:
            window.open()
        with FieldSession(settings, selected) as session:
            session.start()
            while not stopped.is_set():
                try:
                    deliveries = session.deliveries()
                    title = "Sharing" if all(item.received for item in deliveries) else "Connecting / interrupted"
                    message = title + "\n" + "\n".join(f"{item.source}: {item.detail}" for item in deliveries)
                except (httpx.TransportError, ApiError) as error:
                    if isinstance(error, ApiError):
                        session.registry.reauthenticate(error)
                    logger.warning("Remote delivery check failed: %s", error)
                    message = "Remote connection unavailable; retrying. Local sources remain selected."
                if message != previous:
                    print(message, flush=True)
                    previous = message
                if window and not window.update(message):
                    break
                stopped.wait(settings.poll_seconds)
    except (OSError, ValueError, RuntimeError, ApiError) as error:
        if window:
            subprocess.run(["zenity", "--error", "--no-markup", "--title=Sharing stopped", "--text", str(error)])
        raise
    finally:
        if window:
            window.close()


def prepare_phone(settings: FieldSettings, output: Path):
    registry = FieldRegistry(settings)
    try:
        uri = registry.prepare_phone()
    finally:
        registry.close()
    phone = settings.phone
    output.write_text('<!doctype html><meta name="viewport" content="width=device-width">'
        f'<title>{html.escape(phone.callsign)} camera setup</title>'
        '<h1>Phone preparation</h1><p>ATAK supplies operator position and presence. '
        'Keep ATAK connected to the prepared TAK server.</p>'
        f'<p><a href="{html.escape(uri, quote=True)}">Import camera destination into OpenTAK ICU</a></p>'
        '<p>Accept the imported settings and camera permissions once. '
        'In the field: open OpenTAK ICU from ATAK and tap Start Stream.</p>'
        f'<p>Operator UID: {html.escape(phone.tak_uid)}. '
        f'Video ingress: {html.escape(phone.ingress_id)}.</p>')
    output.chmod(0o600)
    print(output.resolve())


def main():
    parser = argparse.ArgumentParser(description="Prepared laptop sharing and ATAK phone setup")
    parser.add_argument("--config", required=True, type=Path)
    commands = parser.add_subparsers(dest="command", required=True)
    launch = commands.add_parser("share")
    launch.add_argument("--headless", action="store_true", help="Share all prepared sources until SIGINT/SIGTERM")
    phone = commands.add_parser("prepare-phone")
    phone.add_argument("--output", type=Path, required=True)
    laptop = commands.add_parser("prepare-laptop")
    laptop.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    settings = FieldSettings.load(args.config)
    if args.command == "share":
        share(settings, args.headless)
    elif args.command == "prepare-phone":
        prepare_phone(settings, args.output)
    else:
        DesktopLauncher(args.config, ROOT).write(args.output)
        print(args.output.resolve())


if __name__ == "__main__":
    main()
