#!/usr/bin/env python3
"""
YOLO detector plugin — runs ultralytics YOLO inference on a video source
and publishes each detection individually on the bus.

Usage:
    from plugins.yolo_detector.yolo_detector import run_plugin
    run_plugin(cfg, bus_config)
"""

import time
import traceback
from typing import Any, Dict, List

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from lib.common import (
    apply_cfg,
    build_envelope,
    build_set_topic,
    build_state_scheduler_topics,
    build_topic_base,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler

LOG = "[YOLO]"


class YoloDetectorPlugin(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)

        # Device selection.
        if self.device == "auto":
            self.device = 0 if torch.cuda.is_available() else "cpu"
        self._use_gpu = self.device != "cpu"

        if self._use_gpu:
            torch.backends.cudnn.benchmark = True

        # Load model.
        self._model = YOLO(self.weights)
        self._names: Dict[int, str] = {}
        try:
            self._names = self._model.model.names
        except Exception:
            self._names = getattr(self._model, "names", {})
        if not isinstance(self._names, dict):
            self._names = {}

        # Optional ByteTrack tracker.
        self._tracker = None
        if self.enable_tracking:
            self._tracker = _load_bytetrack()

        # State scheduler for summary fields.
        base = build_topic_base(self.client_id, self.topic_ns)
        intervals = self.state_intervals
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(
                base,
                {
                    "detection_count": intervals["detection_count"],
                    "model_ready": intervals["model_ready"],
                    "inference_fps": intervals["inference_fps"],
                },
            ),
        )

        # Individual detection publishing — manual rate limit.
        self._det_topic = f"{base}/STATE/detections"
        self._det_interval_s = float(intervals["detections"])
        self._det_last_publish = 0.0

        self.init_bus(self.loop_interval_s)

        # SET topic for runtime config changes.
        self.set_topic = build_set_topic(self.client_id)
        self.add_set_attr(self.set_topic, "confidence", self, "confidence", float)
        self.add_set_attr(self.set_topic, "imgsz", self, "imgsz", int)

        # FPS tracker.
        self._frame_count = 0
        self._fps_window_start = 0.0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _warmup(self) -> None:
        dummy = np.zeros((640, 640, 3), np.uint8)
        self._model(dummy, imgsz=640, conf=0.25, device=self.device, verbose=False)
        if self._use_gpu:
            torch.cuda.synchronize()

    def _infer(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Run YOLO on a frame and return structured detections."""
        results = self._model(
            frame,
            conf=self.confidence,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )[0]

        boxes = results.boxes
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy().astype(int)

        # Optional tracking.
        tracker_ids = None
        if self._tracker is not None:
            try:
                import supervision as sv

                sv_dets = sv.Detections.from_ultralytics(results)
                tracked = self._tracker.update_with_detections(sv_dets)
                if len(tracked) > 0 and tracked.tracker_id is not None:
                    xyxy = tracked.xyxy
                    confs = tracked.confidence if tracked.confidence is not None else confs
                    cls_ids = tracked.class_id if tracked.class_id is not None else cls_ids
                    tracker_ids = tracked.tracker_id
            except Exception:
                pass

        detections: List[Dict[str, Any]] = []
        for i in range(len(xyxy)):
            det: Dict[str, Any] = {
                "class_id": int(cls_ids[i]),
                "class_name": self._names.get(int(cls_ids[i]), str(int(cls_ids[i]))),
                "confidence": round(float(confs[i]), 4),
                "x1": round(float(xyxy[i][0]), 1),
                "y1": round(float(xyxy[i][1]), 1),
                "x2": round(float(xyxy[i][2]), 1),
                "y2": round(float(xyxy[i][3]), 1),
            }
            if tracker_ids is not None:
                det["tracker_id"] = int(tracker_ids[i])
            detections.append(det)

        return detections

    def _publish_detections(self, detections: List[Dict[str, Any]]) -> None:
        """Publish each detection as an individual bus message."""
        now = time.monotonic()
        if self._det_last_publish > 0.0 and now - self._det_last_publish < self._det_interval_s:
            return
        self._det_last_publish = now
        for det in detections:
            envelope = build_envelope(self.client_id, self._det_topic, det)
            self.client.publish(self._det_topic, envelope)

    def _update_fps(self) -> float:
        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_window_start = now
            return round(fps, 1)
        return -1.0  # Sentinel: not ready to report yet.

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        self.send_online()

        # Warm up model.
        print(f"{LOG} id={self.client_id} warming up model weights={self.weights} device={self.device}", flush=True)
        t0 = time.time()
        self._warmup()
        print(f"{LOG} id={self.client_id} warmup done in {time.time() - t0:.2f}s", flush=True)
        self.state_scheduler.update("model_ready", True)
        self.state_scheduler.flush()

        cap = None
        try:
            while True:
                # Open / reopen source.
                cap = self._open_source()
                if cap is None:
                    time.sleep(self.reconnect_delay_s)
                    continue

                print(f"{LOG} id={self.client_id} source={self.source} opened", flush=True)
                self._fps_window_start = time.monotonic()
                self._frame_count = 0

                try:
                    while True:
                        loop_start = time.monotonic()
                        self._pump_once(loop_start + self.loop_interval_s)

                        ok, frame = cap.read()
                        if not ok:
                            print(f"{LOG} id={self.client_id} source ended or dropped", flush=True)
                            break

                        detections = self._infer(frame)
                        if self._use_gpu:
                            torch.cuda.synchronize()

                        self._publish_detections(detections)
                        self.state_scheduler.update("detection_count", len(detections))

                        fps = self._update_fps()
                        if fps >= 0:
                            self.state_scheduler.update("inference_fps", fps)

                        self.state_scheduler.flush()

                        elapsed = time.monotonic() - loop_start
                        sleep_time = self.loop_interval_s - elapsed
                        if sleep_time > 0:
                            time.sleep(sleep_time)

                except (KeyboardInterrupt, SystemExit):
                    raise
                except Exception as exc:
                    print(f"{LOG} id={self.client_id} loop_error={exc}", flush=True)
                finally:
                    cap.release()
                    cap = None

                # If source is a file and not set to loop, we're done.
                if not self.loop_source:
                    break

                time.sleep(self.reconnect_delay_s)

        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        except KeyboardInterrupt:
            pass
        finally:
            if cap is not None:
                cap.release()
            self.state_scheduler.update("model_ready", False)
            self.state_scheduler.flush()
            self.stop()

    def _open_source(self) -> "cv2.VideoCapture | None":
        """Open the configured video source. Returns None on failure."""
        src = self.source
        # Interpret integer strings as camera indices.
        try:
            src = int(src)
        except (ValueError, TypeError):
            pass

        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print(f"{LOG} id={self.client_id} cannot open source={self.source}", flush=True)
            cap.release()
            return None
        return cap


def _load_bytetrack():
    """Import ByteTrack from supervision (handles multiple API versions)."""
    try:
        from supervision.tracker.byte_tracker.core import ByteTrack
        return ByteTrack()
    except Exception:
        pass
    try:
        from supervision import ByteTrack
        return ByteTrack()
    except Exception:
        pass
    print(f"{LOG} ByteTrack not available — tracking disabled", flush=True)
    return None


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    YoloDetectorPlugin(cfg, bus_config).run()
