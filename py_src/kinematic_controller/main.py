#!/usr/bin/env python3
import time
import numpy as np
from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.stepper import step
from kinematic_controller.ik_solver import IK_Solver
from kinematic_controller.gait_engine import GaitEngine
from hardware_abstraction_layer import InputLayer, GamepadController
from kinematic_controller.output import RobotOutput
from hardware_abstraction_layer.dummy_out import DummyOutput

def main():
    gait:Gait = TROT
    gait_detail = gait.steps_in_gait  # (Point/Node count)

    in_layer:InputLayer = GamepadController()
    read_delay_ms = 100

    out_layer:RobotOutput = DummyOutput()

    engine = GaitEngine(gait, out_layer)
    engine.clock_start(50)

    while True:
        input_data = in_layer.poll()

        input_sum = abs(input_data.left_stick.delta_x) + abs(input_data.left_stick.delta_y) + abs(input_data.right_stick.delta_x) + abs(input_data.right_stick.delta_y)
        has_input = input_sum > 0
        if has_input:
            print(f"    L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})    |    R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})")

            engine.input(input_data, 1)

        time.sleep(read_delay_ms / 1000)

    # print(f"#L1_points: {len(gait.time_anchors[0])}; Gait_steps: {gait.steps_in_gait}")  # TODO: Remove, for debugging

    # for i in range(0, gait_detail):
    #     foot_positions = step(gait, i)
    #     print(f"{repr(foot_positions)}")

    #     # Get motor angles
    #     ik = IK_Solver()
    #     result = ik.solve(foot_positions)
    #     if result is None:
    #         return None
    #     print(f"    {repr(result)}")


if __name__ == "__main__":
    main()
