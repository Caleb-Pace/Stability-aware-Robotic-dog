#!/usr/bin/env python3
import time
import threading
import numpy as np
from datetime import datetime

from data_structures.controller_input import Button
from hardware_abstraction_layer import InputLayer, OutputLayer
from hardware_abstraction_layer.gamepad_controller import GamepadController
from hardware_abstraction_layer.unitree_go2_out import UNITREE_SDK_AVAILABLE
from hardware_abstraction_layer.dummy_out import DummyOutput
from hardware_abstraction_layer.simulator import Simulator

from data_structures.gait_definition import Gait
from control.gaits import TROT
from control.actions import *
from control.gait_engine import GaitEngine
from control.pid import PIDController, get_pid_controllers


def main():
    gait:Gait = TROT

    pid_controllers:list[PIDController] = get_pid_controllers()

    interrupt_flag = threading.Event()

    # Controller - Input Interface
    read_delay_ms = 10
    #     Select output layer (Default to Robot controller)
    if UNITREE_SDK_AVAILABLE:
        from hardware_abstraction_layer.unitree_controller import UnitreeController
        in_layer:InputLayer = UnitreeController()
    else:
        in_layer:InputLayer = GamepadController()
    #     Setup button state holders
    a_btn = Button()
    b_btn = Button()
    # y_btn = Button()

    # Output Interface
    #     Select output layer (Default to Robot)
    if UNITREE_SDK_AVAILABLE:
        from hardware_abstraction_layer.unitree_go2_out import UnitreeGo2
        out_layer:OutputLayer = UnitreeGo2()
    else:
        # out_layer:OutputLayer = DummyOutput()
        out_layer:OutputLayer = Simulator(pid_controllers)
    #     Start output layer on separate thread
    output_thread = threading.Thread(target=out_layer.connect, args=(interrupt_flag,))
    output_thread.start()

    # Gait Engine & Clock
    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = read_delay_ms
    clock_thread      = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, interrupt_flag))
    clock_thread.start()

    # State
    is_sitting:bool = True
    bearing:float   = 0.0
    has_performed_test:bool = False

    print("[M ]  Input loop started!")
    while True:
        input_data = in_layer.poll()

        # Buttons
        if a_btn.is_just_pressed(input_data.a_btn_down):
            if is_sitting:
                # engine.perform_action(STAND, 10)
                print(f"[M ]  Stand action triggered!")
            else:
                # engine.perform_action(SIT, 10)
                print(f"[M ]  Sit action triggered!")
            is_sitting = not is_sitting

        if b_btn.is_just_pressed(input_data.b_btn_down):
            engine.perform_action(TURN_RIGHT, 10)
            print(f"[M ]  Turning test triggered!")

        # if y_btn.is_just_pressed(input_data.y_btn_down):
        #     engine.perform_action(TWISTING_TEST, 25)
        #     print(f"[M ]  Twisting test triggered!")
        # if y_btn.is_just_pressed(input_data.y_btn_down):
        #     engine.perform_action(TWISTING_TEST, 25)
        #     print(f"[M ]  Twisting test triggered!")

        # Sticks
        movement_stick_sum = abs(input_data.left_stick.delta_x) + abs(input_data.left_stick.delta_y)
        rotation_stick_sum = abs(input_data.right_stick.delta_x) + abs(input_data.right_stick.delta_y)
        has_input = movement_stick_sum > 0.2  # 0
        if movement_stick_sum > 0.2:
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

        if rotation_stick_sum > 0.2:  # 0
            sensitivity = 5 # TODO: Move
            # print(f"[M ]      R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})")  # TODO: remove, for debugging
            
            delta_x  = input_data.right_stick.delta_x
            yaw      = delta_x * sensitivity
            
            bearing += yaw
            if bearing >= 360 or bearing < 0:
                bearing = bearing % 360
            print(f"Bearing: {round(bearing, 2)}")

            # # Currently unused
            # delta_y = input_data.right_stick.delta_y
            # pitch   = delta_y * sensitivity

            # if not has_performed_test:
            #     has_performed_test = True
            #     engine.rotate(bearing)


        # Delay
        try:
            time.sleep(read_delay_ms / 1000)
        except KeyboardInterrupt:
            break  # Exit loop

    print("[M ]  Input loop finished!")
    interrupt_flag.set()  # Stop the clock thread


if __name__ == "__main__":
    main()
