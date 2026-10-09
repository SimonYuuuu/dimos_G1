# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish the G1 head RealSense RGB stream.

Unitree's ``video_hub_pc4`` service is launched with a numbered V4L2 path
(``/dev/video4`` on this robot). That path changes when the USB camera
re-enumerates, leaving the service alive but permanently returning RPC errors.
This module disables that service through Unitree's public service API and
opens the udev-stable RealSense color-device link directly.
"""

from __future__ import annotations

from glob import glob
import threading
import time
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1VideoHubCameraConfig(ModuleConfig):
    video_interface: str = Field(default="eth0")
    video_device: str | None = None
    video_device_glob: str = "/dev/v4l/by-id/*RealSense*video-index0"
    width: int = Field(default=640, gt=0)
    height: int = Field(default=360, gt=0)
    capture_fps: float = Field(default=15.0, gt=0.0, le=60.0)
    fps: float = Field(default=5.0, gt=0.0, le=15.0)
    jpeg_quality: int = Field(default=80, ge=1, le=100)
    reopen_after_failures: int = Field(default=3, ge=1)
    reopen_backoff: float = Field(default=1.0, ge=0.1)
    frame_id: str = "d435_color_optical_frame"


class G1VideoHubCamera(Module):
    """Capture RGB from the stable RealSense V4L2 link and publish JPEG."""

    config: G1VideoHubCameraConfig
    color_image: Out[CompressedImage]
    dedicated_worker = True

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._capture: Any = None
        self._device: str | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._published_frames = 0
        self._last_frame_bytes = 0
        self._last_error: str | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self._disable_stock_video_hub()
        self._published_frames = 0
        self._last_frame_bytes = 0
        self._last_error = None
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name="g1-realsense-rgb",
        )
        self._thread.start()
        logger.info(
            "G1 RealSense RGB camera starting",
            requested_fps=self.config.fps,
            width=self.config.width,
            height=self.config.height,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        capture = self._capture
        if capture is not None:
            capture.release()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._capture = None
        super().stop()

    @rpc
    def get_camera_status(self) -> dict[str, Any]:
        """Return non-image health data for remote diagnostics."""
        return {
            "thread_alive": self._thread is not None and self._thread.is_alive(),
            "device": self._device,
            "published_frames": self._published_frames,
            "last_frame_bytes": self._last_frame_bytes,
            "last_error": self._last_error,
            "transport": str(self.color_image.transport),
        }

    def _disable_stock_video_hub(self) -> None:
        """Release the RGB device from Unitree's path-fragile stock service."""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

            ChannelFactoryInitialize(0, self.config.video_interface)
            client = RobotStateClient()
            client.SetTimeout(3.0)
            client.Init()
            code = client.ServiceSwitch("video_hub_pc4", False)
            if code != 0:
                logger.warning("Could not disable Unitree video_hub_pc4", code=code)
            else:
                logger.info("Unitree video_hub_pc4 disabled for direct RGB capture")
        except Exception:
            # The camera loop still attempts to open the device; this supports
            # systems where the stock service is absent or already stopped.
            logger.warning("Failed to disable Unitree video_hub_pc4", exc_info=True)

    def _resolve_device(self) -> str:
        if self.config.video_device:
            return self.config.video_device

        candidates = sorted(glob(self.config.video_device_glob))
        if not candidates:
            raise FileNotFoundError(
                f"No RealSense RGB device matched {self.config.video_device_glob!r}"
            )
        return candidates[0]

    def _open_capture(self) -> Any:
        import cv2

        device = self._resolve_device()
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.capture_fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Could not open G1 RGB camera {device}")

        self._device = device
        self._capture = capture
        logger.info("G1 RGB camera opened", device=device)
        return capture

    def _capture_loop(self) -> None:
        import cv2

        publish_period = 1.0 / self.config.fps
        next_publish = 0.0
        consecutive_failures = 0
        capture: Any = None

        while not self._stop_event.is_set():
            if capture is None:
                try:
                    capture = self._open_capture()
                    consecutive_failures = 0
                except Exception as exc:
                    self._last_error = repr(exc)
                    logger.warning("Failed to open G1 RGB camera", exc_info=True)
                    self._stop_event.wait(self.config.reopen_backoff)
                    continue

            ok, frame = capture.read()
            if not ok or frame is None:
                if self._stop_event.is_set():
                    break
                consecutive_failures += 1
                self._last_error = "V4L2 returned no frame"
                if consecutive_failures == 1:
                    logger.warning("G1 RGB camera returned no frame")
                if consecutive_failures >= self.config.reopen_after_failures:
                    capture.release()
                    capture = None
                    self._capture = None
                    self._stop_event.wait(self.config.reopen_backoff)
                continue

            consecutive_failures = 0
            now = time.monotonic()
            if now < next_publish:
                continue

            encoded_ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.config.jpeg_quality],
            )
            if not encoded_ok:
                self._last_error = "JPEG encoding failed"
                logger.warning("Failed to JPEG-encode G1 RGB frame")
                continue

            data = encoded.tobytes()
            self.color_image.publish(
                CompressedImage(data=data, format="jpeg", frame_id=self.config.frame_id)
            )
            self._published_frames += 1
            self._last_frame_bytes = len(data)
            self._last_error = None
            next_publish = now + publish_period
            if self._published_frames == 1:
                logger.info("G1 RGB first frame published", bytes=len(data))

        if capture is not None:
            capture.release()
        self._capture = None
