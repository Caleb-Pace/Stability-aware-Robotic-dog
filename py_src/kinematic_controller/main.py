#!/usr/bin/env python3
import math
from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.stepper import step
from kinematic_controller.ik_solver import IK_Solver

def main():
    gait:Gait = TROT

    print(f"#L1_points: {len(gait.time_anchors[0])}; Gait_steps: {gait.steps_in_gait}")  # TODO: Remove, for debugging
    detail = gait.steps_in_gait  # (Point/Node count)

    for i in range(0, detail):
        foot_positions = step(gait, i)
        print(f"{repr(foot_positions)}")

        # Get motor angles
        ik = IK_Solver()
        result = ik.solve(foot_positions)
        if result is None:
            return None
        print(f"    {repr(result)}")


if __name__ == "__main__":
    main()
