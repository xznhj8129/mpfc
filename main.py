import asyncio
from concurrent.futures import ProcessPoolExecutor
import os
import time
from lib.common import *

CONFIG_PATH = Path(__file__).resolve().with_name("process_config.json")

DATALINK_IN_TOPIC = "Datalink.IN"
DATALINK_OUT_TOPIC = "Datalink.OUT"
CONFIG_ENV = "MAIN_UAV_CONFIG"

def main() -> None:
    env_value = os.environ.get(CONFIG_ENV)
    if not env_value:
        raise EnvironmentError(
            f"Set {CONFIG_ENV} to a config file path (e.g., process_config_msp.json or process_config_mavlink.json)"
        )
    config_path = Path(env_value).resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"config file not found: {config_path}")

    process_config = load_json(config_path)
    bus_config = process_config['bus_config']
    schema_path = bus_config["schema_path"]
    schema = load_json(schema_path)
    topics = schema["topics"]
    main_cfg = process_config["main_uav"]
    corename = main_cfg["core"]