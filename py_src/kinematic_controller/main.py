#!/usr/bin/env python3
import math
from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.stepper import step

def main():
    gait:Gait = TROT

    print(f"#L1_points: {len(gait.time_anchors[0])}; Gait_steps: {gait.steps_in_gait}")  # TODO: Remove, for debugging
    detail = gait.steps_in_gait  # (Point/Node count)

    for i in range(0, detail):
        foot_positions = step(gait, i)
        print(f"{repr(foot_positions)}")

if __name__ == "__main__":
    main()
