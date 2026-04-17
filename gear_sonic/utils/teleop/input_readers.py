"""Input source readers for body tracking data.

PicoReader              -- pulls data from XRoboToolkit SDK (Pico headset).
IsaacTeleopROS2Reader   -- subscribes to IsaacTeleop ROS2 topics as a drop-in replacement.
"""

from collections.abc import Sequence
import logging
import threading
import time
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

try:
    import xrobotoolkit_sdk as xrt
except ImportError:
    xrt = None


def flatten_byte_multi_array_data(data: Sequence[Any]) -> bytes:
    """Flatten `std_msgs/ByteMultiArray.data` into a raw `bytes` payload."""
    if not data:
        return b""

    first = data[0]
    if isinstance(first, int):
        return bytes(data)
    if isinstance(first, (bytes, bytearray, memoryview)):
        return b"".join(bytes(chunk) for chunk in data)

    # Fall back to Python's bytes conversion for array-like containers.
    return bytes(data)


def decode_msgpack_byte_multi_array(
    data: Sequence[Any],
    *,
    msgpack_module,
    msgpack_numpy_module,
) -> dict[str, Any]:
    """Decode a msgpack payload stored in a ROS2 `ByteMultiArray`."""
    payload = flatten_byte_multi_array_data(data)
    if not payload:
        return {}
    return msgpack_module.unpackb(
        payload,
        raw=False,
        object_hook=msgpack_numpy_module.decode,
    )


def build_body_pose_sample(
    data: dict[str, Any],
    *,
    prev_stamp_ns: int | None = None,
    fps_ema: float = 0.0,
) -> tuple[dict[str, Any] | None, int, float]:
    """Convert Isaac Teleop full-body data into the sample schema used by teleop.

    Returns:
        sample: Dict matching the existing `PicoReader.get_latest()` contract,
            or `None` if no joints were available.
        stamp_ns: Latest timestamp to carry forward.
        fps_ema: Updated exponential moving-average FPS estimate.
    """
    positions = data.get("joint_positions") or []
    orientations = data.get("joint_orientations") or []
    n = min(len(positions), len(orientations), 24)
    stamp_ns = int(data.get("timestamp", 0))

    if n == 0:
        return None, stamp_ns, fps_ema

    body_poses = np.zeros((24, 7), dtype=np.float32)
    for idx in range(n):
        body_poses[idx, :3] = np.asarray(positions[idx], dtype=np.float32)
        body_poses[idx, 3:] = np.asarray(orientations[idx], dtype=np.float32)

    device_dt = ((stamp_ns - prev_stamp_ns) * 1e-9) if prev_stamp_ns is not None else 0.0
    if device_dt > 0.0:
        inst = 1.0 / device_dt
        fps_ema = inst if fps_ema == 0.0 else (0.9 * fps_ema + 0.1 * inst)

    sample = {
        "body_poses_np": body_poses,
        "timestamp_realtime": time.time(),
        "timestamp_monotonic": time.monotonic(),
        "timestamp_ns": stamp_ns,
        "dt": device_dt,
        "fps": fps_ema,
    }
    return sample, stamp_ns, fps_ema


class PicoReader:
    """Background reader that pulls Pico/XRT data and computes dt/FPS."""

    STALE_TIMEOUT = 5.0

    def __init__(self, max_queue_size: int = 15):
        del max_queue_size
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._fps_ema = 0.0
        self._last_stamp_ns = None
        self._latest = None
        self._lock = threading.Lock()
        self._last_new_data_time = time.monotonic()
        self._disconnected = threading.Event()

    def start(self):
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def get_latest(self):
        with self._lock:
            return self._latest

    @property
    def disconnected(self) -> bool:
        return self._disconnected.is_set()

    def clear_disconnect(self):
        self._disconnected.clear()
        self._last_new_data_time = time.monotonic()
        self._last_stamp_ns = None
        self._fps_ema = 0.0

    def get_timestamp_ns(self) -> int:
        if xrt is None:
            return 0
        return int(xrt.get_time_stamp_ns())

    def _run(self):
        last_report = time.time()
        while not self._stop.is_set():
            if xrt is None or not xrt.is_body_data_available():
                if (
                    time.monotonic() - self._last_new_data_time > self.STALE_TIMEOUT
                    and not self._disconnected.is_set()
                ):
                    logger.warning(
                        "[PicoReader] No new data for %.1fs, flagging disconnect",
                        self.STALE_TIMEOUT,
                    )
                    self._disconnected.set()
                time.sleep(0.001)
                continue

            stamp_ns = xrt.get_time_stamp_ns()
            prev_stamp_ns = self._last_stamp_ns
            if prev_stamp_ns is not None and stamp_ns == prev_stamp_ns:
                if (
                    time.monotonic() - self._last_new_data_time > self.STALE_TIMEOUT
                    and not self._disconnected.is_set()
                ):
                    logger.warning(
                        "[PicoReader] Timestamps stale for %.1fs, flagging disconnect",
                        self.STALE_TIMEOUT,
                    )
                    self._disconnected.set()
                time.sleep(0.000001)
                continue

            self._last_new_data_time = time.monotonic()
            if self._disconnected.is_set():
                logger.info("[PicoReader] Fresh data received, connection restored")
                self._disconnected.clear()

            device_dt = ((stamp_ns - prev_stamp_ns) * 1e-9) if prev_stamp_ns is not None else 0.0
            if device_dt > 0.0:
                inst = 1.0 / device_dt
                self._fps_ema = inst if self._fps_ema == 0.0 else (0.9 * self._fps_ema + 0.1 * inst)
            self._last_stamp_ns = stamp_ns

            try:
                body_poses = xrt.get_body_joints_pose()
                sample = {
                    "body_poses_np": np.array(body_poses),
                    "timestamp_realtime": time.time(),
                    "timestamp_monotonic": time.monotonic(),
                    "timestamp_ns": stamp_ns,
                    "dt": device_dt,
                    "fps": self._fps_ema,
                }
                with self._lock:
                    self._latest = sample

                now = time.time()
                if now - last_report >= 5.0:
                    logger.info(
                        "[PicoReader] dt_ts: %.2f ms, fps: %.2f",
                        device_dt * 1000.0,
                        self._fps_ema,
                    )
                    last_report = now
            except Exception:
                logger.exception("[PicoReader] read error")


class IsaacTeleopROS2Reader:
    """Background reader that subscribes to Isaac Teleop ROS2 topics."""

    def __init__(
        self,
        _max_queue_size: int = 15,
        full_body_topic: str = "/xr_teleop/full_body",
        controller_topic: str = "/xr_teleop/controller_data",
    ):
        del _max_queue_size

        try:
            import rclpy
            from std_msgs.msg import ByteMultiArray
        except ImportError:
            raise RuntimeError(
                "ROS2 (rclpy) is required for --input-source ros2 but was not found.\n"
                "Install ROS separately with install_scripts/install_ros.sh and source its setup.bash "
                "alongside .venv_teleop."
            ) from None

        import msgpack as _msgpack
        import msgpack_numpy as _msgpack_numpy

        self._stop = threading.Event()
        self._latest = None
        self._latest_controller = None
        self._lock = threading.Lock()
        self._ctrl_lock = threading.Lock()
        self._fps_ema = 0.0
        self._last_stamp_ns = None
        self._msgpack = _msgpack
        self._msgpack_numpy = _msgpack_numpy

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("gear_sonic_ros2_reader")
        self._node.create_subscription(ByteMultiArray, full_body_topic, self._on_full_body, 10)
        self._node.create_subscription(ByteMultiArray, controller_topic, self._on_controller, 10)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._node.get_logger().info(
            f"IsaacTeleopROS2Reader subscribing to {full_body_topic} and {controller_topic}"
        )

    def _decode(self, ros_msg) -> dict[str, Any]:
        return decode_msgpack_byte_multi_array(
            ros_msg.data,
            msgpack_module=self._msgpack,
            msgpack_numpy_module=self._msgpack_numpy,
        )

    def _on_full_body(self, ros_msg) -> None:
        data = self._decode(ros_msg)
        if not data.get("is_active", True):
            return

        sample, stamp_ns, fps_ema = build_body_pose_sample(
            data,
            prev_stamp_ns=self._last_stamp_ns,
            fps_ema=self._fps_ema,
        )
        self._last_stamp_ns = stamp_ns
        self._fps_ema = fps_ema
        if sample is None:
            return

        with self._lock:
            self._latest = sample

    def _on_controller(self, ros_msg) -> None:
        data = self._decode(ros_msg)
        with self._ctrl_lock:
            self._latest_controller = data

    def _spin_loop(self):
        import rclpy

        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.01)

    def start(self):
        if not self._spin_thread.is_alive():
            self._spin_thread.start()

    def stop(self):
        self._stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        try:
            self._node.destroy_node()
        except Exception:
            logger.exception("Failed to destroy ROS2 reader node cleanly")

    def get_latest(self):
        with self._lock:
            return self._latest

    def get_controller_data(self):
        with self._ctrl_lock:
            return self._latest_controller

    @property
    def disconnected(self) -> bool:
        # ROS2 reconnects at the middleware level; the teleop loop just waits for data.
        return False

    def clear_disconnect(self):
        pass

    def get_timestamp_ns(self) -> int:
        with self._lock:
            sample = self._latest
        return int(sample["timestamp_ns"]) if sample else 0
