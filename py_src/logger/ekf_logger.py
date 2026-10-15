"""CSV logging for raw sensors and stability EKF estimates."""

from __future__ import annotations

import csv
import argparse
import logging
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from EKF.stability_ekf import UnitreeGo2StabilityEKF
from hardware_abstraction_layer.output_layer import OutputLayer


LOGGER = logging.getLogger(__name__)


class EKFLogger:
    """Periodically run the stability EKF and record sensors and estimates."""

    FIELDNAMES = (
        "sample", "timestamp_utc", "elapsed_s",
        "accelerometer_x_m_s2", "accelerometer_y_m_s2", "accelerometer_z_m_s2",
        "gyroscope_x_rad_s", "gyroscope_y_rad_s", "gyroscope_z_rad_s",
        "foot_force_fl_n", "foot_force_fr_n", "foot_force_rl_n", "foot_force_rr_n",
        "contact_fl", "contact_fr", "contact_rl", "contact_rr", "contact_count",
        "ekf_roll_deg", "ekf_pitch_deg", "ekf_tilt_deg",
        "ekf_velocity_x_m_s", "ekf_velocity_y_m_s", "ekf_velocity_z_m_s",
        "ekf_cov_roll", "ekf_cov_pitch", "ekf_cov_velocity_x",
        "ekf_cov_velocity_y", "ekf_cov_velocity_z", "ekf_cov_height",
        "ekf_status", "ekf_is_tipping",
    )

    def __init__(
        self,
        output: OutputLayer,
        path: str | Path | None = None,
        sample_interval_s: float = 0.01,
        contact_force_threshold: float = 20.0,
    ):
        if sample_interval_s <= 0:
            raise ValueError("sample_interval_s must be greater than zero")
        if contact_force_threshold < 0:
            raise ValueError("contact_force_threshold must not be negative")

        self.output = output
        self.path = Path(path) if path is not None else self.default_path()
        self.sample_interval_s = float(sample_interval_s)
        self.contact_force_threshold = float(contact_force_threshold)
        self.ekf = UnitreeGo2StabilityEKF(dt=self.sample_interval_s)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._file = None
        self._writer: csv.DictWriter[str] | None = None
        self._start_time = 0.0
        self._sample_count = 0

    @staticmethod
    def default_path(directory: str | Path = "logs") -> Path:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        return Path(directory) / f"ekf_{timestamp}.csv"

    def start(self) -> Path:
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("EKF logger is already running")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()
        self._start_time = time.monotonic()
        self._sample_count = 0
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="ekf-csv-logger", daemon=True)
        self._thread.start()
        LOGGER.info("EKF logging to %s", self.path)
        return self.path

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(1.0, self.sample_interval_s * 5))
        self._thread = None
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

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
        estimate = self.ekf.step_low_state(
            low_state,
            contact_force_threshold=self.contact_force_threshold,
        )
        imu = low_state.imu_state
        accelerometer = self._vector(imu.accelerometer)
        gyroscope = self._vector(imu.gyroscope)
        foot_forces = self._vector4(getattr(low_state, "foot_force", (0.0,) * 4))
        contacts = np.asarray(estimate.foot_contacts, dtype=bool)
        covariance = np.diag(self.ekf.P)

        if self._writer is None or self._file is None:
            raise RuntimeError("EKF logger file is not open")
        self._writer.writerow({
            "sample": self._sample_count,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_s": f"{monotonic_time - self._start_time:.6f}",
            **self._axis_fields("accelerometer", accelerometer, "m_s2"),
            **self._axis_fields("gyroscope", gyroscope, "rad_s"),
            "foot_force_fl_n": f"{foot_forces[0]:.9g}",
            "foot_force_fr_n": f"{foot_forces[1]:.9g}",
            "foot_force_rl_n": f"{foot_forces[2]:.9g}",
            "foot_force_rr_n": f"{foot_forces[3]:.9g}",
            "contact_fl": int(contacts[0]), "contact_fr": int(contacts[1]),
            "contact_rl": int(contacts[2]), "contact_rr": int(contacts[3]),
            "contact_count": estimate.contact_count,
            "ekf_roll_deg": f"{estimate.roll_deg:.9g}",
            "ekf_pitch_deg": f"{estimate.pitch_deg:.9g}",
            "ekf_tilt_deg": f"{estimate.tilt_deg:.9g}",
            "ekf_velocity_x_m_s": f"{estimate.velocity[0]:.9g}",
            "ekf_velocity_y_m_s": f"{estimate.velocity[1]:.9g}",
            "ekf_velocity_z_m_s": f"{estimate.velocity[2]:.9g}",
            "ekf_cov_roll": f"{covariance[0]:.9g}",
            "ekf_cov_pitch": f"{covariance[1]:.9g}",
            "ekf_cov_velocity_x": f"{covariance[2]:.9g}",
            "ekf_cov_velocity_y": f"{covariance[3]:.9g}",
            "ekf_cov_velocity_z": f"{covariance[4]:.9g}",
            "ekf_cov_height": f"{covariance[5]:.9g}",
            "ekf_status": estimate.status.name,
            "ekf_is_tipping": int(estimate.is_tipping),
        })
        self._file.flush()
        self._sample_count += 1

    @staticmethod
    def _axis_fields(prefix: str, values: tuple[float, float, float], unit: str) -> dict[str, str]:
        return {
            f"{prefix}_{axis}_{unit}": f"{value:.9g}"
            for axis, value in zip(("x", "y", "z"), values)
        }

    @staticmethod
    def _vector(values: Any) -> tuple[float, float, float]:
        if values is None or len(values) != 3:
            raise ValueError("IMU vector must contain exactly three values")
        return tuple(float(value) for value in values)

    @staticmethod
    def _vector4(values: Any) -> tuple[float, float, float, float]:
        if values is None or len(values) != 4:
            raise ValueError("Foot force vector must contain exactly four values")
        return tuple(float(value) for value in values)


def main() -> None:
    """Record Unitree sensors and EKF estimates without starting the gait controller."""
    parser = argparse.ArgumentParser(description="Record Go2 EKF data to CSV")
    parser.add_argument("--output", type=Path, default=None, help="CSV output path")
    parser.add_argument("--interval", type=float, default=0.01, help="Sampling interval in seconds")
    parser.add_argument("--duration", type=float, default=None, help="Stop automatically after this many seconds")
    parser.add_argument("--interface", default="eth0", help="Unitree network interface")
    parser.add_argument("--contact-threshold", type=float, default=20.0, help="Contact threshold in newtons")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero")

    try:
        from hardware_abstraction_layer.unitree_go2_out import UNITREE_SDK_AVAILABLE, UnitreeGo2
    except ImportError as error:
        raise RuntimeError("Unitree SDK is required for standalone EKF logging") from error
    if not UNITREE_SDK_AVAILABLE:
        raise RuntimeError("Unitree SDK is required for standalone EKF logging")

    stop_event = threading.Event()
    output = UnitreeGo2(args.interface)
    output_thread = threading.Thread(
        target=output.connect,
        args=(stop_event,),
        name="unitree-ekf-output",
        daemon=True,
    )
    logger = EKFLogger(
        output,
        path=args.output,
        sample_interval_s=args.interval,
        contact_force_threshold=args.contact_threshold,
    )
    output_thread.start()
    path = logger.start()
    print(f"[EKF] Recording to {path}. Press Ctrl-C to stop.")
    try:
        if args.duration is None:
            while not stop_event.wait(0.5):
                pass
        else:
            stop_event.wait(args.duration)
    except KeyboardInterrupt:
        print("\n[EKF] Stopping...")
    finally:
        logger.stop()
        stop_event.set()
        output_thread.join(timeout=5.0)


if __name__ == "__main__":
    main()
