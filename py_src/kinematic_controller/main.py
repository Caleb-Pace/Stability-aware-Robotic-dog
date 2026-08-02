#!/usr/bin/env python3
import time
import threading
import numpy as np

from hardware_abstraction_layer import InputLayer, GamepadController, OutputLayer
from hardware_abstraction_layer.dummy_out import DummyOutput

from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.gait_engine import GaitEngine


def main():
    gait:Gait = TROT

    in_layer:InputLayer = GamepadController()
    read_delay_ms = 10

    out_layer:OutputLayer = DummyOutput()
    out_layer.connect()

    engine = GaitEngine(gait, out_layer)
    clock_interval_ms    = read_delay_ms
    clock_interrupt_flag = threading.Event()
    clock_thread = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, clock_interrupt_flag))
    clock_thread.start()

    while True:
        input_data = in_layer.poll()
        # TODO: Remove, for debugging
        if input_data.a_btn_down:
            print("'A' pressed; ", end="")
        if input_data.b_btn_down:
            print("'B' pressed; ", end="")
        if input_data.x_btn_down:
            print("'X' pressed; ", end="")
        if input_data.y_btn_down:
            print("'Y' pressed; ", end="")
        if input_data.a_btn_down or input_data.b_btn_down or input_data.x_btn_down or input_data.y_btn_down:
            print()

        input_sum = abs(input_data.left_stick.delta_x) + abs(input_data.left_stick.delta_y) + abs(input_data.right_stick.delta_x) + abs(input_data.right_stick.delta_y)
        has_input = input_sum > 0
        if has_input:
            print(f"[M ]      L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})    |    R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})")  # TODO: remove, for debugging

            # engine.input(input_data, 1)

        try:
            time.sleep(read_delay_ms / 1000)
        except KeyboardInterrupt:
            break  # Exit loop

    clock_interrupt_flag.set()  # Stop the clock thread


if __name__ == "__main__":
    main()
