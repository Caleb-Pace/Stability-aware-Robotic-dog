#!/usr/bin/env python3
import time
import threading
import numpy as np
from datetime import datetime

from hardware_abstraction_layer import InputLayer, GamepadController, OutputLayer
from hardware_abstraction_layer.dummy_out import DummyOutput
from hardware_abstraction_layer.simulator import Simulator
from data_structures.controller_input import Button

from data_structures.gait_definition import Gait
from control.gaits import TROT
from control.actions import *
from control.gait_engine import GaitEngine
from control.pid import PIDController, get_pid_controllers


def main():
    gait:Gait = TROT

    # # TODO: Remove, for debugging
    # # print(f"{gait.loop.leg_origins}")
    # print(f"shape: {np.asarray(gait.loop.leg_origins).shape}")
    # print(f"Distance covered by loop: {gait.transition_in.distance_covered}m")
    # print(f"Distance covered by t_in: {gait.loop.distance_covered}m")
    # print(f"Distance covered by t_ou: {gait.transition_out.distance_covered}m")
    
    # # TODO: Remove, for debugging
    # print(f"    steps_count: {SIT.steps_in_gait}")
    # print(f"foot placements: {len(SIT.foot_trajectories[0])}")
    # print(f"       distance: {SIT.distance_covered}")
    # print(f"        origins: {np.asarray(SIT.leg_origins_by_step).shape}\n{np.asarray(SIT.leg_origins_by_step).transpose(1, 0, 2)[0]}")
    
    # # TODO: Remove, for debugging
    # print(f"foot placements: {gait.transition_out.foot_trajectories[-1]}")
    # print(f"foot placements: ", end="")
    # for i in range(4):
    #     # print(f"{gait.transition_out.foot_trajectories[i][-1]}, ", end="")
    #     print(f"{len(gait.transition_out.foot_trajectories[i])}, ", end="")
    # print()
    # return

    pid_controllers:list[PIDController] = get_pid_controllers()

    interrupt_flag = threading.Event()

    read_delay_ms = 10
    in_layer:InputLayer = GamepadController()
    a_btn = Button()

    # out_layer:OutputLayer = DummyOutput()
    out_layer:OutputLayer = Simulator(pid_controllers)
    output_thread = threading.Thread(target=out_layer.connect, args=(interrupt_flag,))
    output_thread.start()

    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = read_delay_ms
    clock_thread      = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, interrupt_flag))
    clock_thread.start()

    # State 
    is_sitting:bool = False
    has_performed_test:bool = False

    print("[M ]  Input loop started!")
    while True:
        input_data = in_layer.poll()

        # Buttons
        if a_btn.is_held(input_data.a_btn_down):
            if is_sitting:
                engine.perform_action(STAND, 10)
                print(f"[M ]  Stand action triggered!")
            else:
                engine.perform_action(SIT, 10)
                print(f"[M ]  Sit action triggered!")
            is_sitting = not is_sitting

        # Sticks
        movement_stick_sum = abs(input_data.left_stick.delta_x) + abs(input_data.left_stick.delta_y)
        rotation_stick_sum = abs(input_data.right_stick.delta_x) + abs(input_data.right_stick.delta_y)
        has_input = movement_stick_sum > 0.2  # 0
        if has_input:
            # print(f"[M ]      L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})    |    R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})")  # TODO: remove, for debugging
            # engine.input(input_data, 1)
            print(f"[M ]      L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})")  # TODO: remove, for debugging

            dist  = 1.0  # m
            speed = 0.5  # m/s

            # # TODO: For test
            # if not has_performed_test:
            #     # engine.perform_action(gait.transition_in, 50)
            #     # print(f"[M ]  t_in done")
            #     # engine.perform_action(gait.loop, 50)
            #     # print(f"[M ]  loop done")
            #     engine.perform_action(gait.transition_out, 50)
            #     print(f"[M ]  t_out done")

            #     has_performed_test = True

            engine.move(dist, speed)
            try:
                time.sleep(dist / speed)
            except KeyboardInterrupt:
                break  # Exit loop


        # Delay
        try:
            time.sleep(read_delay_ms / 1000)
        except KeyboardInterrupt:
            break  # Exit loop
    print("[M ]  Input loop finished!")

    interrupt_flag.set()  # Stop the clock thread


if __name__ == "__main__":
    main()
