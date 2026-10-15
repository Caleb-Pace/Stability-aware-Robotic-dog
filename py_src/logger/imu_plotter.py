"""Plot experiment IMU CSV files."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ACCELEROMETER_COLUMNS = (
    "accelerometer_x_m_s2",
    "accelerometer_y_m_s2",
    "accelerometer_z_m_s2",
)
GYROSCOPE_COLUMNS = (
    "gyroscope_x_rad_s",
    "gyroscope_y_rad_s",
    "gyroscope_z_rad_s",
)
AXIS_LABELS = ("x", "y", "z")


def plot_imu_data(
    csv_path: str | Path,
    output_path: str | Path | None = None,
    show: bool = True,
) -> plt.Figure:
    """Create an accelerometer/gyroscope plot from a logger CSV file."""
    csv_path = Path(csv_path)
    with csv_path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        required_columns = {"elapsed_s", *ACCELEROMETER_COLUMNS, *GYROSCOPE_COLUMNS}
        missing_columns = required_columns - set(reader.fieldnames or ())
        if missing_columns:
            missing = ", ".join(sorted(missing_columns))
            raise ValueError(f"IMU CSV is missing required columns: {missing}")

        rows = list(reader)

    if not rows:
        raise ValueError(f"IMU CSV contains no samples: {csv_path}")

    try:
        elapsed = np.asarray([float(row["elapsed_s"]) for row in rows])
        accelerometer = np.asarray(
            [[float(row[column]) for column in ACCELEROMETER_COLUMNS] for row in rows]
        )
        gyroscope = np.asarray(
            [[float(row[column]) for column in GYROSCOPE_COLUMNS] for row in rows]
        )
    except (TypeError, ValueError) as error:
        raise ValueError(f"IMU CSV contains a non-numeric sample: {csv_path}") from error

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 7), constrained_layout=True)
    for axis, values, title, ylabel in (
        (axes[0], accelerometer, "Accelerometer", "Acceleration (m/s²)"),
        (axes[1], gyroscope, "Gyroscope", "Angular velocity (rad/s)"),
    ):
        for index, label in enumerate(AXIS_LABELS):
            axis.plot(elapsed, values[:, index], label=label)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    axes[1].set_xlabel("Elapsed time (s)")
    figure.suptitle(f"IMU data: {csv_path.name}")

    if output_path is not None:
        figure.savefig(output_path, dpi=150)
    if show:
        plt.show()

    return figure


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot an IMU experiment CSV")
    parser.add_argument("csv_path", type=Path, help="CSV produced by run-dog")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Save the graph to this image path, for example plots/trial.png",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save without opening an interactive window",
    )
    args = parser.parse_args()

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
    plot_imu_data(args.csv_path, output_path=args.save, show=not args.no_show)


if __name__ == "__main__":
    main()
