"""Plot raw sensor, contact, and EKF experiment data."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def plot_ekf_data(csv_path: str | Path, output_path: str | Path | None = None, show: bool = True) -> plt.Figure:
    csv_path = Path(csv_path)
    with csv_path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        required = {
            "elapsed_s", "accelerometer_x_m_s2", "accelerometer_y_m_s2", "accelerometer_z_m_s2",
            "gyroscope_x_rad_s", "gyroscope_y_rad_s", "gyroscope_z_rad_s",
            "contact_fl", "contact_fr", "contact_rl", "contact_rr",
            "ekf_roll_deg", "ekf_pitch_deg", "ekf_tilt_deg", "ekf_status", "ekf_is_tipping",
        }
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"EKF CSV is missing required columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"EKF CSV contains no samples: {csv_path}")

    try:
        time_s = np.asarray([float(row["elapsed_s"]) for row in rows])
        accel = np.asarray([[float(row[f"accelerometer_{axis}_m_s2"]) for axis in "xyz"] for row in rows])
        gyro = np.asarray([[float(row[f"gyroscope_{axis}_rad_s"]) for axis in "xyz"] for row in rows])
        contacts = np.asarray([[int(row[f"contact_{leg}"]) for leg in ("fl", "fr", "rl", "rr")] for row in rows])
        roll = np.asarray([float(row["ekf_roll_deg"]) for row in rows])
        pitch = np.asarray([float(row["ekf_pitch_deg"]) for row in rows])
        tilt = np.asarray([float(row["ekf_tilt_deg"]) for row in rows])
        tipping = np.asarray([int(row["ekf_is_tipping"]) for row in rows])
    except (TypeError, ValueError) as error:
        raise ValueError(f"EKF CSV contains a non-numeric sample: {csv_path}") from error

    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 11), constrained_layout=True)
    for index, axis in enumerate("xyz"):
        axes[0].plot(time_s, accel[:, index], label=axis)
        axes[1].plot(time_s, gyro[:, index], label=axis)
    axes[0].set_title("Raw accelerometer")
    axes[0].set_ylabel("m/s²")
    axes[1].set_title("Raw gyroscope")
    axes[1].set_ylabel("rad/s")
    for axis, title in ((axes[0], "Raw accelerometer"), (axes[1], "Raw gyroscope")):
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    for index, leg in enumerate(("FL", "FR", "RL", "RR")):
        axes[2].step(time_s, contacts[:, index] + index * 1.1, where="post", label=leg)
    axes[2].set_title("Foot contacts")
    axes[2].set_yticks([0, 1.1, 2.2, 3.3], ("FL off", "FR off", "RL off", "RR off"))
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best", ncol=4)

    axes[3].plot(time_s, roll, label="roll")
    axes[3].plot(time_s, pitch, label="pitch")
    axes[3].plot(time_s, tilt, label="tilt", linewidth=2)
    axes[3].fill_between(time_s, 0, 1, where=tipping.astype(bool), alpha=0.15, transform=axes[3].get_xaxis_transform(), label="tipping")
    axes[3].set_title("EKF attitude estimate")
    axes[3].set_ylabel("Degrees")
    axes[3].set_xlabel("Elapsed time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")
    figure.suptitle(f"EKF data: {csv_path.name}")

    if output_path is not None:
        figure.savefig(output_path, dpi=150)
    if show:
        plt.show()
    return figure


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot an EKF experiment CSV")
    parser.add_argument("csv_path", type=Path, help="CSV produced by run-dog")
    parser.add_argument("--save", type=Path, default=None, help="Save the graph to an image path")
    parser.add_argument("--no-show", action="store_true", help="Save without opening a window")
    args = parser.parse_args()
    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
    plot_ekf_data(args.csv_path, args.save, not args.no_show)


if __name__ == "__main__":
    main()
