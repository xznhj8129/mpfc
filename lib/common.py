import json
from message_bus_client import BusClientSync

def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)



cfg = {
    "name": ""
}


def connect(args: argparse.Namespace) -> BusClientSync:
    if args.tcp:
        parts = args.tcp.split(":")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise ValueError("--tcp must be host:port")
        host, port_text = parts
        port = int(port_text)
        if port < 1 or port > 65535:
            raise ValueError("tcp port must be 1-65535")
        return BusClientSync.connect_tcp(host, port, args.id)
    if not args.unix:
        raise ValueError("either --tcp or --unix is required")
    return BusClientSync.connect_unix(args.unix, args.id)