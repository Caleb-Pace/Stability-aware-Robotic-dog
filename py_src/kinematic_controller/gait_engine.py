import time
import numpy as np
from kinematic_controller.gait_definition import Gait
from kinematic_controller.output import RobotOutput
from kinematic_controller.stepper import step


class GaitEngine:
    def __init__(self, gait:Gait, output:RobotOutput):
        self.step:int = 0
        self.delay_ms:float = 1
        
        self.gait = gait
        self.output = output  # Hardware abstraction

    def input(self, delta_x:float) -> None:
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
