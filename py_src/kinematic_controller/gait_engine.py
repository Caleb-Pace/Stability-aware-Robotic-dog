import time
import numpy as np
from data_structures.controller_input import ControllerData, JoyStickData
from kinematic_controller.gait_definition import Gait
from kinematic_controller.output import RobotOutput
from kinematic_controller.stepper import step
from kinematic_controller.ik_solver import IK_Solver


class GaitEngine:
    def __init__(self, gait:Gait, output:RobotOutput):
        self.step_num:int = 0
        self.delay_ms:float = 1
        
        self.gait = gait
        self.output = output  # Hardware abstraction

        self.ik = IK_Solver()


    def clock_tick(self) -> None:
        # self._dynamic_recovery
        # if {recovery} then ignore other calls

        self._step()
        # self._pid
        # combine PID and step results
        pass

    def clock_start(self, interval_ms:float) -> None:
        self.clock_tick()

        try:
            time.sleep(interval_ms / 1000)
        except KeyboardInterrupt:
            pass

        self.clock_start(interval_ms)  # Loop

    def _pid(self, expected, actual) -> None:
        # TODO: Call Maanas' implementation
        pass

    def _step(self):
        foot_positions = step(self.gait, self.step_num)
        motor_angles = self.ik.solve(foot_positions)

        feedforward_torques = self._get_feedforward_torques()  # TODO: Implement properly

        self.output.send_commands(motor_angles, feedforward_torques)

    # _dynamic_recovery

    # TODO: Implement multiplier
    def input(self, controller_data:ControllerData, multiplier:float) -> None:
        if controller_data.left_stick.delta_x > 1:
            self.step_num += 1
        if controller_data.left_stick.delta_x < 1:
            self.step_num -= 1

        self.step_num %= self.gait.steps_in_gait

        # # Crude step delay implementation
        # self.delay_ms = 2 * (1 - abs(controller_data.left_stick.delta_x))  # [0, 2] ms delay
        # time.sleep(self.delay_ms)



    def input2(self, delta_x:float) -> None:
        if delta_x > 1:
            self.step += 1
        if delta_x < 1:
            self.step -= 1
        
        self._update()

        # Crude step delay implementation
        self.delay_ms = 2 * (1 - abs(delta_x))  # [0, 2] ms delay
        time.sleep(self.delay_ms)

    # TODO: Improve update (calling) system
    def _update(self) -> None:
        self.step %= self.gait.steps_in_gait
        foot_positions = step(self.gait, self.step)

        # TODO: Add IK SOLVER
        motor_angles = []
        feedforward_torques = np.zeros(12)  # TODO: Calculate Torques properly

        self.output.send_commands(motor_angles, feedforward_torques)

    def _output(self) -> None:
        pass

    # TODO: Later, dynamics model
    def _get_feedforward_torques(self):
        return np.zeros(12)
    def _dynamic_recovery(self): # -> {Result}|None:
        # is_falling = { Forward Dynamics Solve }
        # if is_falling:  # Save from fall
        #     correction_forces = { Inverse Dynamics Solve }
        #     TODO: Calculate safe/stable position
        #     TODO: You are working in a time frame not instantly moving with those forces
        pass