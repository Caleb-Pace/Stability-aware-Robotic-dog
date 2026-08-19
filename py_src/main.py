#!/usr/bin/env python3
import time
import threading
import numpy as np
from datetime import datetime

from hardware_abstraction_layer import InputLayer, GamepadController, OutputLayer
from hardware_abstraction_layer.dummy_out import DummyOutput
from hardware_abstraction_layer.simulator import Simulator

from data_structures.gait_definition import Gait
from control.gaits import TROT
from control.gait_engine import GaitEngine
from control.pid import PIDController, get_pid_controllers


def main():
    gait:Gait = TROT

    # # TODO: Remove, for debugging
    # print(f"Distance covered by loop: {gait.loop.distance_covered}m")
    # return

    pid_controllers:list[PIDController] = get_pid_controllers()

    interrupt_flag = threading.Event()

    in_layer:InputLayer = GamepadController()
    read_delay_ms = 10

    # out_layer:OutputLayer = DummyOutput()
    out_layer:OutputLayer = Simulator(pid_controllers)
    output_thread = threading.Thread(target=out_layer.connect, args=(interrupt_flag,))
    output_thread.start()

    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = read_delay_ms
    clock_thread      = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, interrupt_flag))
    clock_thread.start()

    a_btn_was_down:bool = False  # TODO: Extract button state logic into its own class
    press_count = 0

    print("[M ]  Input loop started!")
    while True:
        input_data = in_layer.poll()
        # TODO: Remove, for debugging
        if input_data.a_btn_down and ( not a_btn_was_down ):
            a_btn_was_down = True

            print(f"{datetime.now().strftime("%Y-%m-%d %H:%M:S.%f")} 'A' pressed; {press_count}")
            press_count += 1
            # out_layer.send_stand_action()
        elif ( not input_data.a_btn_down ) and a_btn_was_down:
            a_btn_was_down = False

        input_sum = abs(input_data.left_stick.delta_x) + abs(input_data.left_stick.delta_y) + abs(input_data.right_stick.delta_x) + abs(input_data.right_stick.delta_y)
        has_input = input_sum > 0
        if has_input:
            print(f"[M ]      L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})    |    R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})")  # TODO: remove, for debugging

            engine.input(input_data, 1)

        try:
            time.sleep(read_delay_ms / 1000)
        except KeyboardInterrupt:
            break  # Exit loop
    print("[M ]  Input loop finished!")

    interrupt_flag.set()  # Stop the clock thread


if __name__ == "__main__":
    main()
