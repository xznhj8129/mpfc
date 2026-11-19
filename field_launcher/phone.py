"""OpenTAK ICU's native settings-import URI (Preferences / OnBoardingActivity)."""
from typing import Literal
from urllib.parse import urlencode

from field_launcher.config import Configuration


class ICUSettings(Configuration):
    protocol: Literal["srt"] = "srt"
    address: str
    port: int
    path: str
    username: str = ""
    password: str = ""

    def import_uri(self) -> str:
        return "opentakicu://import?" + urlencode(self.model_dump())
