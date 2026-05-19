import numpy as np
from kinematic_controller.gait_definition import Gait
from kinematic_controller.output import RobotOutput
from kinematic_controller.stepper import step


class GaitEngine:
    def __init__(self, gait:Gait, output:RobotOutput):
        self.step = 0
        self.gait = gait
        self.output = output  # Hardware abstraction

    def input(self, delta_x:float):
        if delta_x > 1:
            self.step += 1
        if delta_x < 1:
            self.step -= 1

        self._update()

    def _update(self):
        self.step %= self.gait.steps_in_gait
        foot_positions = step(self.gait, self.step)

        # TODO: Add IK SOLVER
        motor_angles = []
        feedforward_torques = np.zeros(12)  # TODO: Calculate Torques properly

        self.output.send_commands(motor_angles, feedforward_torques)