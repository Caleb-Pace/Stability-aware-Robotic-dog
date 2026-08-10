import time
import threading
import numpy as np

from data_structures import Action
from data_structures.controller_input import ControllerData

from hardware_abstraction_layer import OutputLayer

from data_structures.gait_definition import Gait, LEG_COUNT
from control.stepper import step
from kinematics.ik_solver import IK_Solver
from kinematics.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT


class GaitEngine:
    def __init__(self, gait:Gait, output:OutputLayer):
        self._last_step_num:int = -1
        self._step_num:int = 0
        
        self.gait = gait
        self.output = output  # Hardware abstraction

        self._ik = IK_Solver()


    def _clock_tick(self) -> None:
        # self._dynamic_recovery
        # if {recovery} then ignore other calls

        if self._step_num != self._last_step_num:
            self._step()
            self._last_step_num = self._step_num
        # self._pid
        # combine PID and step results
        pass

    def clock_start(self, interval_ms:float, interrupt:threading.Event) -> None:
        while not interrupt.is_set():
            self._clock_tick()

            try:
                time.sleep(interval_ms / 1000)
            except KeyboardInterrupt:
                pass

    def _pid(self, expected, actual) -> None:
        # TODO: Call Maanas' implementation
        pass

    def _step(self):
        foot_positions = step(self.gait, self._step_num)
        target_motor_angles = self._ik.solve(foot_positions)

        feedforward_torques = self._get_feedforward_torques()  # TODO: Implement properly

        action = Action(target_motor_angles, feedforward_torques)
        self.output.send_action(action)



    def _convert_distance_to_steps(self):
        # Take:
        #     Gait,
        #     Current step number,
        #     Target distance
        # Return:
        #     Steps to take to achieve distance
        pass

    def move(self, distance:float, speed:float):
        # 1. Calculate pushing steps (negative x, with ground contact z = 0)
        
        # 2. Calculate steps needed to achieve that distance
        
        # 3. Calculate how long movement should be (distance / speed)
        movement_time = distance / speed  # seconds

        # 4. Calculate time per step to fit that timeframe (step_interval_ms)
        step_interval_ms = 0


        

    # TODO: Implement multiplier
    def input(self, controller_data:ControllerData, multiplier:float) -> None:
        if controller_data.left_stick.delta_y > 0:
            self._step_num += 1
        if controller_data.left_stick.delta_y < 0:
            self._step_num -= 1

        self._step_num %= self.gait.steps_in_gait
        print(f"[GE]  {self._step_num}    (L-d_y: {np.round(controller_data.left_stick.delta_y, 3):>6})")  # TODO: remove, for debugging


    # TODO: Later, dynamics model
    # _dynamic_recovery
    def _get_feedforward_torques(self):
        return np.array(([_HIP_ABDUCTOR_TORQUE_LIMIT.maximum, _HIP_TORQUE_LIMIT.maximum, _KNEE_TORQUE_LIMIT.maximum] * LEG_COUNT), dtype=np.float64)
    def _dynamic_recovery(self): # -> {Result}|None:
        # is_falling = { Forward Dynamics Solve }
        # if is_falling:  # Save from fall
        #     correction_forces = { Inverse Dynamics Solve }
        #     TODO: Calculate safe/stable position
        #     TODO: You are working in a time frame not instantly moving with those forces
        pass