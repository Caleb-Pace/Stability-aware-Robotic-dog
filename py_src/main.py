#!/usr/bin/env python3
import time
import threading
import argparse
import numpy as np

from data_structures.controller_input import Button
from data_structures.controller_input import JOYSTICK_DEADZONE
from hardware_abstraction_layer import InputLayer, OutputLayer
from hardware_abstraction_layer.gamepad_controller import GamepadController
from hardware_abstraction_layer.unitree_go2_out import UNITREE_SDK_AVAILABLE
from hardware_abstraction_layer.dummy_out import DummyOutput
from hardware_abstraction_layer.simulator import Simulator

from data_structures.gait_definition import Gait
from control.gaits import TROT, CRAWL
from control.actions import *
from control.gait_engine import GaitEngine
from control.pid import PIDController, get_pid_controllers
from logger import IMULogger
from logger import EKFLogger

try:
    from EKF.go2_lowcmd_recovery_node import Go2LowLevelRecoveryNode
except ImportError:
        Go2LowLevelRecoveryNode = None


def main():
    parser = argparse.ArgumentParser(description="Run the stability-aware Go2 controller")
    parser.add_argument(
        "--imu-log",
        default=None,
        help="CSV path for IMU data (default: logs/imu_<UTC timestamp>.csv)",
    )
    parser.add_argument(
        "--imu-interval",
        type=float,
        default=0.01,
        help="IMU logging interval in seconds (default: 0.01)",
    )
    parser.add_argument("--ekf-log", default=None, help="CSV path for EKF experiment data")
    parser.add_argument("--ekf-interval", type=float, default=0.01, help="EKF logging interval in seconds")
    parser.add_argument(
        "--ekf-contact-threshold",
        type=float,
        default=20.0,
        help="Foot-force threshold in newtons for EKF contact detection (default: 20)",
    )
    args = parser.parse_args()

    gait:Gait = TROT
    # gait:Gait = CRAWL

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
    y_btn = Button()

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
    imu_logger = IMULogger(
        output=out_layer,
        path=args.imu_log,
        sample_interval_s=args.imu_interval,
    )
    imu_log_path = imu_logger.start()
    print(f"[M ]  IMU logging to {imu_log_path}")
    ekf_logger = EKFLogger(
        out_layer,
        path=args.ekf_log,
        sample_interval_s=args.ekf_interval,
        contact_force_threshold=args.ekf_contact_threshold,
    )
    ekf_log_path = ekf_logger.start()
    print(f"[M ]  EKF logging to {ekf_log_path}")

    # Gait Engine & Clock
    engine = GaitEngine(gait, out_layer)
    clock_interval_ms = read_delay_ms
    clock_thread      = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, interrupt_flag))
    clock_thread.start()

    # EFK / Stability
    recovery_thread = None
    recovery_node = None
    if UNITREE_SDK_AVAILABLE and Go2LowLevelRecoveryNode is not None:
        recovery_node = Go2LowLevelRecoveryNode(output=out_layer, engine=engine, manage_output=False)
        recovery_thread = threading.Thread(
            target=recovery_node.run,
            name="go2-low-level-recovery",
        )
        recovery_thread.start()

    # State
    is_sitting:bool = True
    motion_was_active:bool = False

    print("[M ]  Input loop started!")
    try:
        while True:
            input_data = in_layer.poll()

            # Buttons
            if a_btn.is_just_pressed(input_data.a_btn_down):
                if is_sitting:
                    engine.perform_action(STAND, 10)
                    print(f"[M ]  Stand action triggered!")
                else:
                    engine.perform_action(SIT, 10)
                    print(f"[M ]  Sit action triggered!")
                is_sitting = not is_sitting

            if b_btn.is_just_pressed(input_data.b_btn_down):
                engine.HEHE = not engine.HEHE
                print(f"[M ]  Toggled Dynamics {('ON' if engine.HEHE else 'off')}!")
                # engine.perform_action(HALF_STAND, 10)
                # print(f"[M ]  Half stand triggered!")
                # engine.perform_action(TURN_RIGHT, 10)
                # print(f"[M ]  Turning test triggered!")

            if y_btn.is_just_pressed(input_data.y_btn_down) and recovery_node is not None:
                recovery_node.toggle_ekf()
                print(f"[M ]  Toggled EKF {('ON' if recovery_node.ekf_enabled else 'off')}!")


            # Sticks
            # Sticks are continuous velocity-style commands. The gait engine
            # advances while the stick is deflected and returns to the default
            # standing pose as soon as the command returns to the deadzone.
            if is_sitting:
                motion_was_active = False
            else:
                command = (
                    input_data.left_stick.delta_y,
                    0,
                    input_data.right_stick.delta_x,
                )
                command_is_active = float(np.linalg.norm(command)) > max(
                    0.1,
                    JOYSTICK_DEADZONE,
                )
                if command_is_active:
                    engine.set_motion(*command)  # TODO: Add speed scale
                    motion_was_active = True
                elif motion_was_active:
                    engine.stop_motion()
                    engine.return_to_stand()
                    print("[M ]  Joystick released; returning to standing pose")
                    motion_was_active = False


            time.sleep(read_delay_ms / 1000)
    except KeyboardInterrupt:
        print("\n[M ]  Stopping...")
    finally:
        interrupt_flag.set()
        imu_logger.stop()
        ekf_logger.stop()
        if recovery_node is not None:
            recovery_node.stop()
        print("[M ]  Input loop finished!")


if __name__ == "__main__":
    main()
