
# filepath: /home/maanas/Desktop/Projects/Stability-aware-Robotic-dog/py_src/main.py
#!/usr/bin/env python3
import argparse
import os
import time
import threading
import numpy as np
from datetime import datetime

from hardware_abstraction_layer import InputLayer, GamepadController, OutputLayer
from hardware_abstraction_layer.dummy_out import DummyOutput
from hardware_abstraction_layer.simulator import Simulator
from hardware_abstraction_layer.robot_out import UnitreeGo2Output

from data_structures.gait_definition import Gait
from control.gaits import TROT
from control.gait_engine import GaitEngine
from control.pid import PIDController, get_pid_controllers


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--backend",
        choices=("simulator", "dummy", "robot"),
        default=os.getenv("DOG_BACKEND", "simulator"),
    )
    parser.add_argument(
        "--interface",
        default=os.getenv("DOG_NETWORK_INTERFACE", "eth0"),
        help="Robot network interface for Unitree mode",
    )
    return parser.parse_args()


def build_output_layer(backend: str, pid_controllers):
    if backend == "dummy":
        return DummyOutput()
    if backend == "robot":
        return UnitreeGo2Output(network_interface=backend_args.interface)
    return Simulator(pid_controllers)


def main():
    global backend_args
    backend_args = parse_args()

    gait: Gait = TROT
    pid_controllers: list[PIDController] = get_pid_controllers()

    interrupt_flag = threading.Event()

    in_layer: InputLayer = GamepadController()
    read_delay_ms = 10

    out_layer: OutputLayer
    if backend_args.backend == "dummy":
        out_layer = DummyOutput()
    elif backend_args.backend == "robot":
        from hardware_abstraction_layer.robot_out import UnitreeGo2Output
        out_layer = UnitreeGo2Output(network_interface=backend_args.interface)
    else:
        out_layer = Simulator(pid_controllers)

    output_thread = threading.Thread(target=out_layer.connect, args=(interrupt_flag,))
    output_thread.start()

    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = read_delay_ms
    clock_thread = threading.Thread(
        target=engine.clock_start,
        args=(clock_interval_ms, interrupt_flag),
    )
    clock_thread.start()

    a_btn_was_down: bool = False
    press_count = 0

    print("[M ]  Input loop started!")
    try:
        while True:
            input_data = in_layer.poll()

            if input_data.a_btn_down and not a_btn_was_down:
                a_btn_was_down = True
                print(f"[M ]  {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')} 'A' pressed; {press_count}")
                press_count += 1
            elif not input_data.a_btn_down and a_btn_was_down:
                a_btn_was_down = False

            input_sum = (
                abs(input_data.left_stick.delta_x)
                + abs(input_data.left_stick.delta_y)
                + abs(input_data.right_stick.delta_x)
                + abs(input_data.right_stick.delta_y)
            )
            has_input = input_sum > 0
            if has_input:
                print(
                    f"[M ]      L ({np.round(input_data.left_stick.delta_x, 3):>6}, {np.round(input_data.left_stick.delta_y, 3):>6})"
                    f"    |    R ({np.round(input_data.right_stick.delta_x, 3):>6}, {np.round(input_data.right_stick.delta_y, 3):>6})"
                )
                engine.input(input_data, 1)

            time.sleep(read_delay_ms / 1000)
    except KeyboardInterrupt:
        pass

    print("[M ]  Input loop finished!")
    interrupt_flag.set()


if __name__ == "__main__":
    main()