import time
import threading
import math
import numpy as np

from data_structures import Position
from data_structures.controller_input import ControllerData
from data_structures.leg_trajectories import LegTrajectories
from typing import Tuple

from hardware_abstraction_layer import OutputLayer
from queue import Queue

from data_structures.gait_definition import Gait, LEG_COUNT
from control.stepper import step
from kinematics.ik_solver import IK_Solver
from kinematics.ik_solver import _HIP_ABDUCTOR_TORQUE_LIMIT, _HIP_TORQUE_LIMIT, _KNEE_TORQUE_LIMIT


class Instruction:
    DEFAULT_STEP_INTERVAL_MS = 5

    step_interval_ms:float = DEFAULT_STEP_INTERVAL_MS
    trajectory:LegTrajectories
    repeat:int

    def __init__(self, instruction:LegTrajectories, step_interval_ms:float|None = None, repeat:int = 1):
        if step_interval_ms:
            self.step_interval_ms = step_interval_ms

        self.trajectory = instruction
        self.repeat     = repeat

class GaitEngine:
    _instruction_queue = Queue()
    _current_instruction:Instruction|None = None
    # _current_pose =  # Stores current position of all end-effectors
    _clock_interval_ms:float = 5

    _last_position:Position|None = None

    gait:Gait
    _current_bearing:float = 0.0


    def __init__(self, gait:Gait, output:OutputLayer):
        self._last_step_num:int = -1
        self._step_num:int = 0
        
        self.gait = gait
        self.output = output  # Hardware abstraction

        self._ik = IK_Solver()


    def _send_position(self):
        if self._current_instruction is None:
            return  # Early exit: no work to do

        trajectory = self._current_instruction.trajectory

        # Retrieve position
        # print(f"[GE][DEBUG]:59  get origin: {self._step_num}/{len(trajectory.leg_origins_by_step)}")  # TODO: Remove, for debugging
        hip_origins         = trajectory.leg_origins_by_step[self._step_num]
        foot_positions      = step(trajectory, self._step_num)
        # Calculate motor angles
        target_motor_angles = self._ik.solve(foot_positions, hip_origins)
        if None in target_motor_angles:
            return  # Early exit: IK - Failed

        feedforward_torques = self._get_feedforward_torques()  # TODO: Implement properly

        # Send position
        position = Position(target_motor_angles, feedforward_torques)
        self.output.send_position(position)
        self._last_position = position

    def _step(self):
        # # TODO: Remove, for debugging
        # if not (self._current_instruction is None and self._instruction_queue.empty()):
        #     print(f"[GE]  _step() call | got instruction? {self._current_instruction is not None} | {self._instruction_queue.qsize()} instructions queued")
        
        if self._current_instruction is None:  # Request new instruction
            if self._instruction_queue.empty():

                # Maintain last position
                if self._last_position:
                    self.output.send_position(self._last_position)

                return # Early exit: no work to do

            # New instruction
            self._current_instruction = self._instruction_queue.get()
            self._step_num            = 0
            #     Adjust clock time interval (for speed)
            if self._current_instruction:
                print(f"[GE]  new step interval {self._current_instruction.step_interval_ms}ms")  # TODO: Remove, for debugging
                self._clock_interval_ms = self._current_instruction.step_interval_ms

        # Work instruction
        self._send_position()
        self._step_num += 1

        # Check if instruction finished
        if self._current_instruction:
            if not ( self._step_num >= self._current_instruction.trajectory.steps_in_gait ):
                return  # Work to do

            # Performed instruction
            print(f"[GE][DEBUG]:96  instruction complete")  # TODO: Remove, for debugging
            # time.sleep(0.5)  # TODO: Remove, for testing
            self._current_instruction.repeat -= 1
            if self._current_instruction.repeat <= 0:  # Clear finished instruction
                self._current_instruction = None
            else:
                self._step_num = 0  # Reset step count


    def _clock_tick(self) -> None:
        # self._dynamic_recovery
        # if {recovery} then ignore other calls

        self._step()

        # # TODO: Remove, for use with old input system
        # if self._step_num != self._last_step_num:
        #     self._step_old()
        #     self._last_step_num = self._step_num
        
        # self._pid
        # combine PID and step results

    def clock_start(self, interval_ms:float, interrupt:threading.Event) -> None:
        self._clock_interval_ms = interval_ms

        while not interrupt.is_set():
            self._clock_tick()

            try:
                time.sleep(self._clock_interval_ms / 1000)
            except KeyboardInterrupt:
                pass


    def clear_instruction_queue(self):
        self._instruction_queue.queue.clear()

    # # TODO: Need some sort of method, action to bring feet back to ground, for stabilisation.
    # def abort(self):
    #     self.clear_instruction_queue()
    #     # TODO: Add stablise method


    def perform_action(self, action:LegTrajectories, step_interval_ms:float|None = None):
        instruction = Instruction(action, step_interval_ms)
        self._instruction_queue.put(instruction)
        print(f"[GE]  action requested")  # TODO: Remove, for debugging

    def _calculate_move_distance_data(self, distance:float, speed:float) -> Tuple[float, int]:
        # 1. Handle gait transitions
        distance -= (self.gait.transition_in.distance_covered + self.gait.transition_out.distance_covered)

        # 2. Calculate pushing steps (negative x, with ground contact z = 0)
        cycle_distance = self.gait.loop.distance_covered

        # 3. Calculate steps needed to achieve that distance
        print(f"[GE][DEBUG]:139 cycles = round({distance} / {cycle_distance})")
        cycles = round(distance / cycle_distance)
        steps  = (cycles * self.gait.loop.steps_in_gait)

        # 4. Calculate how long movement should be (distance / speed)
        movement_time = distance / speed  # seconds

        # 5. Calculate time per step to fit that timeframe (step_interval)
        step_interval    = movement_time / steps  # seconds / steps
        step_interval_ms = step_interval * 1_000 

        print(f"[GE]  move() | {{Distance}} target: {distance}m, best: {cycles * cycle_distance}m ({cycles} * {cycle_distance}) | {{Speed}} {speed}m/s (step interval: {step_interval_ms}ms)")  # TODO: Remove, for debugging
        return (step_interval_ms, cycles)
    
    def rotate(self, bearing:float):  # TODO: Note, potentially could add speed parameter.
        yaw_change = bearing - self._current_bearing

        sample_count:int = math.ceil(  abs(yaw_change) * 5 )  # Scale step count
        rotation = LegTrajectories(
            sample_count       = sample_count,
            heights            = np.array([0.3], dtype=float),
            orientations       = np.array([  # orientation: [roll, pitch, yaw] in radians
                [0, 0, 0]#, [0, 0, yaw_change]
            ], dtype=float),
            leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
            leg_control_points = np.array([
                np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
                np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
                np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
                np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            ], dtype=object)
        )

        self._instruction_queue.put( Instruction(rotation) )

    def move(self, distance:float, speed:float):
        step_interval_ms, repeat_loop = self._calculate_move_distance_data(distance, speed)
        loop_instruction = Instruction(self.gait.loop, step_interval_ms, repeat_loop)
        
        self._instruction_queue.put( Instruction(self.gait.transition_in, step_interval_ms) )
        self._instruction_queue.put( loop_instruction )
        self._instruction_queue.put( Instruction(self.gait.transition_out, step_interval_ms) )
        print(f"[GE]  move({distance}m, {speed}m/s) requested")  # TODO: Remove, for debugging

    def move_direction(self, distance:float, bearing:float, speed:float):
        step_interval_ms, repeat_loop = self._calculate_move_distance_data(distance, speed)
        pass
    def move2(self, distance_x:float, distance_y:float, speed:float):
        pass





    def _pid(self, expected, actual) -> None:
        # TODO: Call Maanas' implementation
        pass

    # def _step_old(self):
    #     trajectory:LegTrajectories|None = self.gait.loop
    #     # trajectory:LegTrajectories|None = None

    #     # Action support
    #     if self._current_instruction is not None:
    #         if self._step_num == self._current_instruction.trajectory.steps_in_gait: # Action finished
    #             self._current_action = None
    #             self._step_num       = 0
    #         else:
    #             trajectory = self._current_instruction.trajectory
    #     # print(f"[GE]  has trajectory? {trajectory is not None}")  # TODO: Remove, for debugging

    #     if trajectory:
    #         # Retrieve position
    #         foot_positions = step(trajectory, self._step_num)
    #         target_motor_angles = self._ik.solve(foot_positions, trajectory.leg_origins_by_step[self._step_num])
    #         if None in target_motor_angles:
    #             return  # Early exit: IK - Failed

    #         feedforward_torques = self._get_feedforward_torques()  # TODO: Implement properly

    #         # Send position
    #         position = Position(target_motor_angles, feedforward_torques)
    #         self.output.send_position(position)

    # # TODO: Implement multiplier
    # def input(self, controller_data:ControllerData, multiplier:float) -> None:
    #     if controller_data.left_stick.delta_y > 0:
    #         self._step_num += 1
    #     if controller_data.left_stick.delta_y < 0:
    #         self._step_num -= 1

    #     self._step_num %= self.gait.loop.steps_in_gait
    #     print(f"[GE]  {self._step_num}    (L-d_y: {np.round(controller_data.left_stick.delta_y, 3):>6})")  # TODO: remove, for debugging


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