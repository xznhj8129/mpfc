from __future__ import annotations

import subprocess
import sys
import unittest
import uuid
from pathlib import Path

from lib.occid_bus import occid, unpack_occid
from lib.uav_client import UavClient


TARGET_UID = occid.UID(root=uuid.UUID("11111111-1111-4111-8111-111111111111").bytes)


class FakeBus:
    def __init__(self) -> None:
        self.client_id = "program"
        self.request_counter = 0
        self.published: list[tuple[str, dict]] = []

    def publish(self, topic: str, payload: dict) -> None:
        self.published.append((topic, payload))


class FakeRuntime:
    def __init__(self) -> None:
        self.bus = FakeBus()
        self.bus_config = {"topic_prefix": "mpfc/uav1"}
        self.state = {}
        self.wait_calls: list[tuple[str, float]] = []

    def _wait_response(self, request_id: str, timeout_s: float) -> dict:
        self.wait_calls.append((request_id, timeout_s))
        return {"request_id": request_id, "ok": True, "data": {}}


class UavClientRoutingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = FakeRuntime()
        self.uav = UavClient(
            self.runtime,
            {"id": "uav_controller", "topic_ns": "UAV"},
            response_timeout_s=10.0,
            target_uid=TARGET_UID,
        )

    def test_convenience_arm_maps_to_state_change_without_waiting(self) -> None:
        request_id = self.uav.arm()
        self.assertEqual(request_id, "req-1")
        self.assertEqual(self.runtime.wait_calls, [])
        topic, envelope = self.runtime.bus.published[-1]
        self.assertEqual(topic, "uav_controller/UAV/REQUEST")
        command = envelope["data"]["command"]
        self.assertEqual(envelope["data"]["request_id"], "req-1")
        self.assertEqual(command["_occid_model"], "StateChangeCommand")
        self.assertEqual(command["operation"], "SET")
        self.assertEqual(command["property_name"], "armed")
        self.assertTrue(command["value"]["bool"])
        target = unpack_occid(command["target_uid"])
        self.assertEqual(bytes(target.root), bytes(TARGET_UID.root))

    def test_execute_waits_only_when_explicitly_requested(self) -> None:
        response = self.uav.execute(self.uav.arm_command(), timeout_s=3.0)
        self.assertTrue(response["ok"])
        self.assertEqual(self.runtime.wait_calls, [("req-1", 3.0)])

    def test_go_to_maps_to_motion_command(self) -> None:
        self.uav.go_to(45.0, -73.0, 20.0)
        command = self.runtime.bus.published[-1][1]["data"]["command"]
        self.assertEqual(command["_occid_model"], "MotionCommand")
        self.assertEqual(command["operation"], "MOVE_TO")
        self.assertEqual(command["destination"]["lat"], 45.0)

    def test_high_rate_attitude_uses_input_path_without_request_id(self) -> None:
        self.uav.set_attitude(0.1, -0.2, 0.3, 0.4)
        topic, envelope = self.runtime.bus.published[-1]
        self.assertEqual(topic, "uav_controller/UAV/INPUT")
        self.assertEqual(envelope["data"]["_occid_model"], "ControlAttitudeSetpoint")
        self.assertNotIn("request_id", envelope["data"])
        self.assertEqual(self.runtime.wait_calls, [])

    def test_generic_command_is_not_accepted_by_uav_service(self) -> None:
        command = occid.Command(target_uid=TARGET_UID, constraints=[])
        with self.assertRaises(TypeError):
            self.uav.send(command)

    def test_wrong_target_is_rejected(self) -> None:
        command = occid.StateChangeCommand(
            target_uid=occid.UID(root=uuid.uuid4().bytes),
            constraints=[],
            operation=occid.StateChangeOperation.SET,
            property_name="armed",
            value=occid.MetadataValue(bool=True),
        )
        with self.assertRaisesRegex(ValueError, "target_uid"):
            self.uav.send(command)

    def test_direct_control_lifecycle_uses_process_control(self) -> None:
        begin = self.uav.begin_direct_control("ATTITUDE_THRUST")
        end = self.uav.end_direct_control()
        self.assertTrue(begin["ok"])
        self.assertTrue(end["ok"])
        self.assertEqual(self.runtime.wait_calls, [("req-1", 10.0), ("req-2", 10.0)])
        commands = [item[1]["data"]["command"] for item in self.runtime.bus.published]
        self.assertEqual(
            [command["_occid_model"] for command in commands],
            ["ProcessControlCommand", "ProcessControlCommand"],
        )
        self.assertEqual(commands[0]["operation"], "START")
        self.assertEqual(commands[0]["process_name"], "direct_control.attitude_thrust")
        self.assertEqual(commands[1]["operation"], "STOP")
        self.assertEqual(commands[1]["process_name"], "direct_control")


class RuntimeImportTests(unittest.TestCase):
    def test_main_prioritizes_mpfc_lib_when_occid_path_is_first(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        occid_root = repo_root.parent / "occid"
        code = "\n".join(
            [
                "import runpy, sys",
                f"sys.path.insert(0, {str(occid_root)!r})",
                f"runpy.run_path({str(repo_root / 'main.py')!r}, run_name='mpfc_import_probe')",
                "import lib.common",
                f"assert lib.common.__file__.startswith({str(repo_root)!r}), lib.common.__file__",
            ]
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
