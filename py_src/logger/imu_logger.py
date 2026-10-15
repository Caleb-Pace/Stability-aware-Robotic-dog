"""CSV logging for the Go2 IMU exposed by an output layer."""

from __future__ import annotations

import csv
import argparse
import logging
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from hardware_abstraction_layer.output_layer import OutputLayer


LOGGER = logging.getLogger(__name__)


class IMULogger:
    """Periodically record IMU samples from an :class:`OutputLayer`.

    The logger writes a header as soon as it is started and flushes every
    sample, so a stopped experiment leaves a usable CSV even after an
    unexpected process exit. Output layers that have not initialized a
    low-level state yet are supported; sampling begins when state is available.
    """

    FIELDNAMES = (
        "sample",
        "timestamp_utc",
        "elapsed_s",
        "accelerometer_x_m_s2",
        "accelerometer_y_m_s2",
        "accelerometer_z_m_s2",
        "gyroscope_x_rad_s",
        "gyroscope_y_rad_s",
        "gyroscope_z_rad_s",
    )

    def __init__(
        self,
        output: OutputLayer,
        path: str | Path | None = None,
        sample_interval_s: float = 0.01,
    ):
        if sample_interval_s <= 0:
            raise ValueError("sample_interval_s must be greater than zero")

        self.output = output
        self.path = Path(path) if path is not None else self.default_path()
        self.sample_interval_s = float(sample_interval_s)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._file = None
        self._writer: csv.DictWriter[str] | None = None
        self._start_time = 0.0
        self._sample_count = 0

    @staticmethod
    def default_path(directory: str | Path = "logs") -> Path:
        """Return a timestamped path without overwriting another experiment."""
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        return Path(directory) / f"imu_{timestamp}.csv"

    def start(self) -> Path:
        """Create the CSV and start sampling in a background thread."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("IMU logger is already running")

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()
        self._start_time = time.monotonic()
        self._sample_count = 0
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="imu-csv-logger",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info("IMU logging to %s", self.path)
        return self.path

    def stop(self) -> None:
        """Stop sampling and close the CSV file."""
        self._stop_event.set()
        if self._thread is not None and threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(1.0, self.sample_interval_s * 5))
            if self._thread.is_alive():
                LOGGER.warning("IMU logger did not stop within its timeout")
        self._thread = None
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

    def __enter__(self) -> "IMULogger":
        self.start()
        return self

    def __exit__(self, *_: Any) -> None:
        self.stop()

    def _run(self) -> None:
        next_sample = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_sample:
                low_state = self.output.get_low_state()
                if low_state is not None:
                    self._write_sample(low_state, now)
                next_sample += self.sample_interval_s
                if next_sample < now:
                    next_sample = now + self.sample_interval_s
            self._stop_event.wait(max(0.0, next_sample - time.monotonic()))

    def _write_sample(self, low_state: Any, monotonic_time: float) -> None:
        imu = getattr(low_state, "imu_state", None)
        if imu is None:
            LOGGER.warning("Low state has no imu_state; skipping sample")
            return

        accelerometer = self._vector(imu, "accelerometer")
        gyroscope = self._vector(imu, "gyroscope")
        assert self._writer is not None
        assert self._file is not None
        self._writer.writerow(
            {
                "sample": self._sample_count,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_s": f"{monotonic_time - self._start_time:.6f}",
                "accelerometer_x_m_s2": f"{accelerometer[0]:.9g}",
                "accelerometer_y_m_s2": f"{accelerometer[1]:.9g}",
                "accelerometer_z_m_s2": f"{accelerometer[2]:.9g}",
                "gyroscope_x_rad_s": f"{gyroscope[0]:.9g}",
                "gyroscope_y_rad_s": f"{gyroscope[1]:.9g}",
                "gyroscope_z_rad_s": f"{gyroscope[2]:.9g}",
            }
        )
        self._file.flush()
        self._sample_count += 1

    @staticmethod
    def _vector(imu: Any, name: str) -> tuple[float, float, float]:
        values = getattr(imu, name, None)
        if values is None or len(values) != 3:
            raise ValueError(f"IMU {name} must contain exactly three values")
        return (float(values[0]), float(values[1]), float(values[2]))


def main() -> None:
    """Record the Unitree low-state IMU without starting the gait controller."""
    parser = argparse.ArgumentParser(description="Record Go2 IMU data to CSV")
    parser.add_argument("--output", type=Path, default=None, help="CSV output path")
    parser.add_argument("--interval", type=float, default=0.01, help="Sampling interval in seconds")
    parser.add_argument("--duration", type=float, default=None, help="Stop automatically after this many seconds")
    parser.add_argument("--interface", default="eth0", help="Unitree network interface")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero")

    try:
        from hardware_abstraction_layer.unitree_go2_out import UNITREE_SDK_AVAILABLE, UnitreeGo2
    except ImportError as error:
        raise RuntimeError("Unitree SDK is required for standalone IMU logging") from error
    if not UNITREE_SDK_AVAILABLE:
        raise RuntimeError("Unitree SDK is required for standalone IMU logging")

    stop_event = threading.Event()
    output = UnitreeGo2(args.interface)
    output_thread = threading.Thread(
        target=output.connect,
        args=(stop_event,),
        name="unitree-imu-output",
        daemon=True,
    )
    logger = IMULogger(output, path=args.output, sample_interval_s=args.interval)
    output_thread.start()
    path = logger.start()
    print(f"[IMU] Recording to {path}. Press Ctrl-C to stop.")
    try:
        if args.duration is None:
            while not stop_event.wait(0.5):
                pass
        else:
            stop_event.wait(args.duration)
    except KeyboardInterrupt:
        print("\n[IMU] Stopping...")
    finally:
        logger.stop()
        stop_event.set()
        output_thread.join(timeout=5.0)


if __name__ == "__main__":
    main()
