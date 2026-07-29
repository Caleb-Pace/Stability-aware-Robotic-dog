import time
import numpy as np
from kinematic_controller.gait_definition import Gait
from kinematic_controller.output import RobotOutput
from kinematic_controller.stepper import step


class GaitEngine:
    def __init__(self, gait:Gait, output:RobotOutput):
        self.step_num:int = 0
        self.delay_ms:float = 1
        
        self.gait = gait
        self.output = output  # Hardware abstraction

    def clock_tick(self) -> None:
        # self._dynamic_recovery
        # if {recovery} then ignore other calls

        # self._step
        # self._pid
        # combine PID and step results
        pass

    def clock_start(self, interval_ms:float) -> None:
        self.clock_tick()
        time.sleep(interval_ms)
        self.clock_start(interval_ms)

    def _pid(self, expected, actual) -> None:
        # TODO: Call Maanas' implementation
        pass

    def _step(self):
        pass

    # _dynamic_recovery

    def input(self) -> None:
        pass


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
        pass
    def _dynamic_recovery(self): # -> {Result}|None:
        # is_falling = { Forward Dynamics Solve }
        # if is_falling:  # Save from fall
        #     correction_forces = { Inverse Dynamics Solve }
        #     TODO: Calculate safe/stable position
        #     TODO: You are working in a time frame not instantly moving with those forces
        pass