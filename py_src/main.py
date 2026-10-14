#!/usr/bin/env python3
import time
import threading
import numpy as np
from datetime import datetime

from data_structures.controller_input import Button
from data_structures.controller_input import JOYSTICK_DEADZONE
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

try:
    from EKF.go2_lowcmd_recovery_node import Go2LowLevelRecoveryNode
except ImportError:
    try:
        from py_src.EKF.go2_lowcmd_recovery_node import Go2LowLevelRecoveryNode
    except ImportError:
        Go2LowLevelRecoveryNode = None


# TODO: Remove, for debugging
def _print_indexes(traj:LegTrajectories, loop_ref:LegTrajectories|None = None):
    for step_num in range(traj.steps_in_gait):
        debug_str = f"{{{step_num:>2}}}  |"

        if loop_ref is None:
            for leg in range(4):
                offset = traj.phase_offsets_as_steps[leg]

                # Calculate new point/step index
                index = offset + step_num
                if index >= traj.steps_in_gait:  # Wrap index if necessary
                    index %= traj.steps_in_gait

                debug_str += f" {str( index ):<2} |"
        else:
            for leg in range(4):
                target_point = traj.foot_trajectories[leg][step_num]
                leg_points   = loop_ref.foot_trajectories[leg]

                match_mask = (leg_points == target_point).all(axis=1)
                index      = np.where(match_mask)[0][0]

                debug_str += f" {str( index ):<2} |"
        
        print(debug_str)

def _print_values(traj:LegTrajectories):
    import control.stepper as stepper

    for step_num in range(traj.steps_in_gait):
        foot_points = stepper.step(traj, step_num)

        debug_str = f"{{{step_num:>2}}}  |"
        for leg in range(4):
            debug_str += f" {str( np.round(foot_points[leg], 6) ):<31} |"
        print(debug_str)

def _print_origins(traj:LegTrajectories):
    for step_num in range(traj.steps_in_gait):
        origins = traj.leg_origins_by_step[step_num]

        debug_str = f"{{{step_num:>2}}}  |"
        for leg in range(4):
            debug_str += f" {str( np.round(origins[leg], 6) ):<31} |"
        print(debug_str)


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
    x_btn = Button()
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

    # Gait Engine & Clock
    engine = GaitEngine(gait, out_layer)
    # engine.perform_action(SIT, 10)
    clock_interval_ms = read_delay_ms
    clock_thread      = threading.Thread(target=engine.clock_start, args=(clock_interval_ms, interrupt_flag))
    clock_thread.start()

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
    motion_was_active = False

    print("[M ]  Input loop started!")
    while True:
        input_data = in_layer.poll()

        # Buttons
        if a_btn.is_just_pressed(input_data.a_btn_down):
            engine.stop_motion()
            if is_sitting:
                engine.perform_action(STAND, 10)
                print(f"[M ]  Stand action triggered!")
            else:
                engine.perform_action(SIT, 10)
                print(f"[M ]  Sit action triggered!")
            is_sitting = not is_sitting

        if b_btn.is_just_pressed(input_data.b_btn_down):
            engine.stop_motion()
            engine.perform_action(TURN_RIGHT, 10)
            print(f"[M ]  Turning test triggered!")

        if x_btn.is_just_pressed(input_data.x_btn_down) and isinstance(out_layer, Simulator):
            engine.stop_motion()
            out_layer.reset_state()
            is_sitting = True
            motion_was_active = False
            print("[M ]  Simulator reset to startup state")

        if y_btn.is_just_pressed(input_data.y_btn_down) and recovery_node is not None:
            recovery_node.toggle_ekf()

        # Sticks are continuous velocity-style commands. The gait engine
        # advances while the stick is deflected and returns to the default
        # standing pose as soon as the command returns to the deadzone.
        if is_sitting:
            motion_was_active = False
        else:
            command = (
                input_data.left_stick.delta_y,
                input_data.left_stick.delta_x,
                input_data.right_stick.delta_x,
            )
            # Keep the release state latched through small controller readings.
            # Without this threshold, stick noise can alternate between motion
            # and return-to-stand on successive polls.
            command_is_active = float(np.linalg.norm(command)) > max(
                0.1,
                JOYSTICK_DEADZONE,
            )
            if command_is_active:
                engine.set_motion(*command)
                motion_was_active = True
            elif motion_was_active:
                engine.stop_motion()
                engine.return_to_stand()
                print("[M ]  Joystick released; returning to standing pose")
                motion_was_active = False


        # Delay
        try:
            time.sleep(read_delay_ms / 1000)
        except KeyboardInterrupt:
            break  # Exit loop

    print("[M ]  Input loop finished!")
    interrupt_flag.set()  # Stop the clock thread
    if recovery_node is not None:
        recovery_node.stop()
    if recovery_thread is not None and recovery_thread.is_alive():
        recovery_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
