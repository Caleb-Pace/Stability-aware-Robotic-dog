#!/usr/bin/env python3
import argparse
import os
import time
import threading
import numpy as np

from hardware_abstraction_layer import InputLayer, GamepadController, OutputLayer
from hardware_abstraction_layer.dummy_out import DummyOutput
from hardware_abstraction_layer.robot_output import UnitreeGo2Output
from hardware_abstraction_layer.simulator import Simulator

from data_structures.gait_definition import Gait
from control.gaits import TROT
from control.gait_engine import GaitEngine
from control.pid import PIDController, get_pid_controllers


def _parse_args():
    parser = argparse.ArgumentParser(description="Run the robot controller")
    parser.add_argument(
        "--backend",
        choices=("robot", "simulator", "dummy"),
        default=os.getenv("DOG_BACKEND", "robot"),
        help="Select the hardware abstraction backend",
    )
    parser.add_argument(
        "--interface",
        default=os.getenv("DOG_NETWORK_INTERFACE", "eth0"),
        help="Network interface used by the robot backend",
    )
    return parser.parse_args()


def main():
    args = _parse_args()

    gait:Gait = TROT

    pid_controllers:list[PIDController] = get_pid_controllers()

    interrupt_flag = threading.Event()

    in_layer:InputLayer = GamepadController()
    read_delay_ms = 100

    if args.backend == "dummy":
        out_layer:OutputLayer = DummyOutput()
    elif args.backend == "simulator":
        out_layer = Simulator(pid_controllers)
    else:
        out_layer = UnitreeGo2Output(network_interface=args.interface)

    output_thread = threading.Thread(target=out_layer.connect, args=(interrupt_flag,))
    output_thread.start()

    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = read_delay_ms
    clock_thread      = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, interrupt_flag))
    clock_thread.start()

    a_btn_down_prev:bool = False

    print("[M ]  Input loop started!")
    while True:
        input_data = in_layer.poll()
        # TODO: Remove, for debugging
        if input_data.a_btn_down:
            print("'A' pressed; ")
            out_layer.send_stand_action()

        input_sum = abs(input_data.left_stick.delta_x) + abs(input_data.left_stick.delta_y) + abs(input_data.right_stick.delta_x) + abs(input_data.right_stick.delta_y)
        has_input = input_sum > 0
        if has_input:
            print(f"[M ]      L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})    |    R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})")  # TODO: remove, for debugging

            # engine.input(input_data, 1)

        try:
            time.sleep(read_delay_ms / 1000)
        except KeyboardInterrupt:
            break  # Exit loop
    print("[M ]  Input loop finished!")

    interrupt_flag.set()  # Stop the clock thread


if __name__ == "__main__":
    main()
