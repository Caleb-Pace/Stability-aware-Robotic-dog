import time
import threading
import math
import numpy as np

from data_structures import Position, Point3DList
from data_structures.leg_trajectories import LegTrajectories
from data_structures.constants import LEG_COUNT, JOINT_COUNT, STAND_HEIGHT
from typing import Tuple

from hardware_abstraction_layer import OutputLayer
from queue import Queue

from data_structures.constants import LEG_COUNT, JOINT_COUNT
from data_structures.gait_definition import Gait
from control.stepper import step
from kinematics.ik_solver import IK_Solver

from control.actions import STAND

from control.dynamics import BasicDynamicsModel

from control.gaits import LEAN_FORWARD_X, RETURN_TO_STAND_LIFT_HEIGHT, PITCH_COMP


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
    STRAFE_STEPS = 176
    STRAFE_LIFT_STEPS = 32
    STRAFE_STEP_WIDTH = 0.20
    STRAFE_LIFT_HEIGHT = 0.16
    STRAFE_HIP_SHIFT = 0.05
    STRAFE_SETTLE_STEPS = 8

    _instruction_queue:Queue
    _current_instruction:Instruction|None
    # _current_pose =  # Stores current position of all end-effectors
    _clock_interval_ms:float

    _last_position:Position|None = None

    gait:Gait
    _current_bearing:float = 0.0

    # Direct input support
    _motion_command: np.ndarray
    _motion_lock: threading.Lock
    _continuous_motion_active: bool
    _last_motion_foot_positions: np.ndarray | None
    _motion_start_step: int
    _motion_start_steps: int
    _motion_lift_steps: int
    _returning_to_stand: bool

    HEHE:bool = True


    def __init__(self, gait:Gait, output:OutputLayer):
        self._instruction_queue = Queue()
        self._current_instruction = None
        self._clock_interval_ms = Instruction.DEFAULT_STEP_INTERVAL_MS

        self._last_step_num:int = -1
        self._step_num:int = 0
        
        self.gait = gait
        self.output = output  # Hardware abstraction

        self._ik = IK_Solver()
        self._dynamics = BasicDynamicsModel()

        self._motion_command = np.zeros(3, dtype=np.float64)
        self._motion_lock = threading.Lock()
        self._continuous_motion_active = False
        self._last_motion_foot_positions = None
        self._motion_start_step = 0
        self._motion_start_steps = 24
        self._motion_lift_steps = 6
        self._returning_to_stand = False


    def _send_position(self):
        if self._current_instruction is None:
            return  # Early exit: no work to do

        trajectory = self._current_instruction.trajectory

        # Retrieve position
        # print(f"[GE][DEBUG]:59  get origin: {self._step_num}/{len(trajectory.leg_origins_by_step)}")  # TODO: Remove, for debugging
        hip_origins         = np.asarray( trajectory.leg_origins_by_step[self._step_num] )
        foot_positions      = step(trajectory, self._step_num)
        # Calculate motor angles
        target_motor_angles = self._ik.solve(foot_positions, hip_origins)
        if None in target_motor_angles:
            return  # Early exit: IK - Failed

        feedforward_torques = self._get_feedforward_torques(
            foot_positions,
            hip_origins,
            target_motor_angles,
        )
        # for leg in range(4):
        #     print(f"{leg}, {target_motor_angles[leg][0]}, {target_motor_angles[leg][1]}, {target_motor_angles[leg][2]}")  # TODO: Remove, for debugging

        # Send position
        position = Position(target_motor_angles, feedforward_torques)
        self.output.send_position(position)
        self._last_position = position

    def _send_motion_position(self) -> None:
        """Send one phase of the continuously commanded gait with correct 
        forward walking and corrected turn direction.
        """
        trajectory = self.gait.loop

        with self._motion_lock:
            forward, lateral, yaw = self._motion_command

        command_magnitude = float(np.linalg.norm([forward, lateral, yaw]))
        if command_magnitude == 0.0:
            return

        if abs(lateral) > max(abs(forward), abs(yaw)):
            self._step_num %= self.STRAFE_STEPS
            self._send_strafe_position(lateral)
            return

        self._step_num %= trajectory.steps_in_gait
        foot_positions = step(trajectory, self._step_num).copy()

        stride_profile = foot_positions[:, 0].copy()
        
        # Corrected side_sign for turning right (clockwise):
        # FR (-1), FL (+1), RR (-1), RL (+1)
        side_sign = np.array([-1.0, 1.0, -1.0, 1.0])
        
        foot_positions[:, 0] = stride_profile * (
            forward / command_magnitude + yaw / command_magnitude * side_sign
        )
        foot_positions[:, 1] += stride_profile * lateral / command_magnitude

        # Lean: full at pure-forward, 0 for backward
        forward_fraction = forward / command_magnitude          # -1 .. +1
        lean_weight = max(0.0, forward_fraction)                # 1 forward, 0 back/side
        foot_positions[:, 0] += LEAN_FORWARD_X * lean_weight

        if self._motion_start_step < self._motion_start_steps:
            stride_phase = (self._motion_start_step + 1) / self._motion_start_steps
            stride_blend = stride_phase * stride_phase * (3.0 - 2.0 * stride_phase)

            # Establish swing-foot clearance before applying horizontal stride.
            lift_phase = min(
                1.0,
                (self._motion_start_step + 1) / self._motion_lift_steps,
            )
            lift_blend = lift_phase * lift_phase * (3.0 - 2.0 * lift_phase)
            foot_positions[:, :2] *= stride_blend
            foot_positions[:, 2] *= lift_blend
            self._motion_start_step += 1

        hip_origins = np.asarray(trajectory.leg_origins_by_step[self._step_num])
        self._last_motion_foot_positions = foot_positions.copy()
        target_motor_angles = self._ik.solve(foot_positions, hip_origins)
        if None in target_motor_angles:
            return

        position = Position(
            target_motor_angles,
            self._get_feedforward_torques(
                foot_positions,
                hip_origins,
                target_motor_angles,
            ),
        )
        self.output.send_position(position)
        self._last_position = position

    def _send_strafe_position(self, lateral: float) -> None:
        """Move laterally in two planted pairs without dragging a foot.

        The pair on the side of travel moves first and remains planted while
        the opposite pair is lifted and moved to the same lateral target.
        Every horizontal foot displacement is performed while that foot is
        above the ground.
        """
        phase = self._step_num % self.STRAFE_STEPS
        # Match the controller's positive horizontal axis to body-frame Y.
        direction = float(np.sign(lateral))
        target_y = direction * self.STRAFE_STEP_WIDTH
        foot_positions = np.zeros((LEG_COUNT, 3), dtype=np.float64)

        destination_legs = (1, 3) if direction > 0.0 else (0, 2)
        opposite_legs = (0, 2) if direction > 0.0 else (1, 3)

        def move_pair(
            legs: tuple[int, int],
            start_step: int,
            move_step: int,
        ) -> None:
            progress = np.clip(
                (phase - start_step) / self.STRAFE_LIFT_STEPS,
                0.0,
                1.0,
            )
            smooth_progress = progress * progress * (3.0 - 2.0 * progress)
            for leg in legs:
                foot_positions[leg, 1] = target_y * smooth_progress
                foot_positions[leg, 2] = self.STRAFE_LIFT_HEIGHT * (
                    smooth_progress if phase < move_step else 1.0 - smooth_progress
                )

        move_pair(destination_legs, 0, self.STRAFE_LIFT_STEPS)
        if phase >= self.STRAFE_LIFT_STEPS:
            for leg in destination_legs:
                foot_positions[leg, 1] = target_y
                foot_positions[leg, 2] = 0.0

        second_pair_start = 72
        if phase >= second_pair_start:
            move_pair(
                opposite_legs,
                second_pair_start,
                second_pair_start + self.STRAFE_LIFT_STEPS,
            )
            if phase >= second_pair_start + self.STRAFE_LIFT_STEPS:
                for leg in opposite_legs:
                    foot_positions[leg, 1] = target_y
                    foot_positions[leg, 2] = 0.0

        def reset_pair(legs: tuple[int, int], start_step: int) -> None:
            if phase < start_step:
                return
            progress = np.clip(
                (phase - start_step) / self.STRAFE_LIFT_STEPS,
                0.0,
                1.0,
            )
            smooth_progress = progress * progress * (3.0 - 2.0 * progress)
            lift = self.STRAFE_LIFT_HEIGHT * np.sin(np.pi * progress)
            for leg in legs:
                foot_positions[leg, 1] = target_y * (1.0 - smooth_progress)
                foot_positions[leg, 2] = lift

        # Let the second pair finish and settle before recentring either pair.
        destination_reset_start = (
            second_pair_start
            + self.STRAFE_LIFT_STEPS
            + self.STRAFE_SETTLE_STEPS
        )
        opposite_reset_start = (
            destination_reset_start + self.STRAFE_LIFT_STEPS
            - self.STRAFE_SETTLE_STEPS
        )
        reset_pair(destination_legs, destination_reset_start)
        reset_pair(opposite_legs, opposite_reset_start)

        trajectory = self.gait.loop
        hip_origins = np.asarray(
            trajectory.leg_origins_by_step[
                self._step_num % trajectory.steps_in_gait
            ]
        ).copy()
        hip_origins[:, 1] += direction * self.STRAFE_HIP_SHIFT
        self._last_motion_foot_positions = foot_positions.copy()
        target_motor_angles = self._ik.solve(foot_positions, hip_origins)
        if None in target_motor_angles:
            return

        position = Position(
            target_motor_angles,
            self._get_feedforward_torques(
                foot_positions,
                hip_origins,
                target_motor_angles,
            ),
        )
        self.output.send_position(position)
        self._last_position = position

    def _step(self):
        with self._motion_lock:
            continuous_motion = bool(np.any(self._motion_command))

        if continuous_motion and self._current_instruction is None:
            self._send_motion_position()
            with self._motion_lock:
                forward, lateral, yaw = self._motion_command
            strafe_active = abs(lateral) > max(abs(forward), abs(yaw))
            phase_steps = (
                self.STRAFE_STEPS
                if strafe_active
                else self.gait.loop.steps_in_gait
            )
            self._step_num = (self._step_num + 1) % phase_steps
            return

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
                # print(f"[GE]  new step interval {self._current_instruction.step_interval_ms}ms")  # TODO: Remove, for debugging
                self._clock_interval_ms = self._current_instruction.step_interval_ms

        # Work instruction
        self._send_position()
        self._step_num += 1

        # Check if instruction finished
        if self._current_instruction:
            if not ( self._step_num >= self._current_instruction.trajectory.steps_in_gait ):
                return  # Work to do

            # Performed instruction
            # print(f"[GE][DEBUG]:96  instruction complete")  # TODO: Remove, for debugging
            # time.sleep(0.5)  # TODO: Remove, for testing
            self._current_instruction.repeat -= 1
            if self._current_instruction.repeat <= 0:  # Clear finished instruction
                self._current_instruction = None
                self._returning_to_stand = False
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


    # TODO: FLAG, calculates loop time.
    # TODO: Add scale to make faster
    def set_motion(self, forward: float, lateral: float, yaw: float) -> None:
        """Update continuous joystick motion without queueing a fixed move.

        Values are normalized to approximately [-1, 1]. The caller handles
        the zero-command transition to the desired neutral pose.
        """
        command = np.clip(
            np.asarray([forward, lateral, yaw], dtype=np.float64),
            -1.0,
            1.0,
        )
        with self._motion_lock:
            starting_motion = not self._continuous_motion_active and bool(np.any(command))
            self._motion_command = command
            self._continuous_motion_active = bool(np.any(command))

        if starting_motion and self._current_instruction is None:
            self._step_num = 0
            self._motion_start_step = 0

        magnitude = float(np.linalg.norm(command))
        if magnitude > 0.0:
            if abs(command[1]) > max(abs(command[0]), abs(command[2])):
                # Strafe phases need time to transfer weight and settle.
                strafe_frequency_hz = 0.35 + 0.65 * min(1.0, abs(command[1]))
                self._clock_interval_ms = 1000.0 / (
                    self.STRAFE_STEPS * strafe_frequency_hz
                )
                return
            frequency_hz = 1.0 + 3.0 * min(1.0, magnitude)
            self._clock_interval_ms = 1000.0 / (
                self.gait.loop.steps_in_gait * frequency_hz
            )
            # print(f"[GE]  motion - update {frequency_hz}hz")  # TODO: Remove, for debugging

    def stop_motion(self) -> None:
        """Stop continuous gait advancement and clear the motion command."""
        with self._motion_lock:
            self._motion_command.fill(0.0)
            self._continuous_motion_active = False
        self._clock_interval_ms = Instruction.DEFAULT_STEP_INTERVAL_MS

    # TODO: FLAG, transition to stand time.
    def return_to_stand(self, step_interval_ms: float = 10.0) -> None:
        """Ease from current motion foot targets into neutral standing 
        quickly and smoothly with alternating diagonal stomps.
        """
        if self._returning_to_stand:
            return

        start_positions = self._last_motion_foot_positions
        if start_positions is None:
            self.perform_action(STAND, step_interval_ms)
            return

        # Retrieve true neutral standing foot positions from the STAND action
        neutral_stand_positions = step(STAND, 0)

        sample_count = 24
        phase = np.linspace(0.0, 1.0, sample_count)
        control_points = np.empty(LEG_COUNT, dtype=object)

        for leg, start in enumerate(start_positions):
            target = neutral_stand_positions[leg]
            
            # Diagonal pairs: FR/RL (legs 0 and 3) and FL/RR (legs 1 and 2)
            pair_delay = 0.0 if leg in (0, 3) else 0.2
            pair_duration = 0.65
            
            # Normalize local phase for this leg pair
            pair_phase = np.clip((phase - pair_delay) / pair_duration, 0.0, 1.0)
            
            # Smoothstep easing for lift and lower phases
            lift_progress = np.clip(pair_phase / 0.3, 0.0, 1.0)
            lift_progress = lift_progress * lift_progress * (3.0 - 2.0 * lift_progress)
            
            lower_progress = np.clip((pair_phase - 0.7) / 0.3, 0.0, 1.0)
            lower_progress = lower_progress * lower_progress * (3.0 - 2.0 * lower_progress)
            
            max_start_z = max(start[2], target[2])
            z_profile = np.empty(sample_count)
            
            for i, p in enumerate(pair_phase):
                if p <= 0.3:
                    # Step 1: Lift up from current height
                    z_profile[i] = start[2] + RETURN_TO_STAND_LIFT_HEIGHT * lift_progress[i]
                elif p <= 0.7:
                    # Step 2: Hold elevated height while shifting horizontally
                    z_profile[i] = max_start_z + RETURN_TO_STAND_LIFT_HEIGHT
                else:
                    # Step 3: Lower down smoothly to neutral stand target Z
                    z_profile[i] = (max_start_z + RETURN_TO_STAND_LIFT_HEIGHT) * (1.0 - lower_progress[i]) + target[2] * lower_progress[i]
            
            # Horizontal movement phase (occurs entirely while foot is elevated)
            move_phase = np.clip((pair_phase - 0.25) / 0.5, 0.0, 1.0)
            move_phase = move_phase * move_phase * (3.0 - 2.0 * move_phase)
            
            points = np.empty((sample_count, 3), dtype=float)
            for i in range(sample_count):
                # Smoothly interpolate X and Y from current stop position to neutral stand target
                points[i, 0] = start[0] * (1.0 - move_phase[i]) + target[0] * move_phase[i]
                points[i, 1] = start[1] * (1.0 - move_phase[i]) + target[1] * move_phase[i]
                points[i, 2] = z_profile[i]
                
            control_points[leg] = points

        transition = LegTrajectories(
            sample_count=sample_count,
            heights=np.array([STAND_HEIGHT], dtype=float),
            orientations=np.array([[0.0, PITCH_COMP, 0.0]], dtype=float),
            leg_phase_offset=np.zeros(LEG_COUNT, dtype=float),
            leg_control_points=control_points,
            should_interpolate=False,
        )
        neutral_origins = np.asarray(STAND.leg_origins_by_step[-1])
        transition.leg_origins_by_step = [
            neutral_origins.copy() for _ in range(transition.steps_in_gait)
        ]
        self._returning_to_stand = True
        self.perform_action(transition, step_interval_ms)


    def clear_instruction_queue(self):
        self._instruction_queue.queue.clear()

    # # TODO: Need some sort of method, action to bring feet back to ground, for stabilisation.
    # def abort(self):
    #     self.clear_instruction_queue()
    #     # TODO: Add stablise method

    def perform_action(self, action:LegTrajectories, step_interval_ms:float|None = None):
        instruction = Instruction(action, step_interval_ms)
        self._instruction_queue.put(instruction)
        # print(f"[GE]  action requested")  # TODO: Remove, for debugging


    def _calculate_move_distance_data(self, distance:float, speed:float) -> Tuple[float, int]:
        # 1. Handle gait transitions
        if self.gait.transition_in:
            distance -= self.gait.transition_in.distance_covered
        if self.gait.transition_out:
            distance -= self.gait.transition_out.distance_covered

        # 2. Calculate pushing steps (negative x, with ground contact z = 0)
        cycle_distance = self.gait.loop.distance_covered

        # 3. Calculate steps needed to achieve that distance
        # print(f"[GE][DEBUG]:139 cycles = round({distance} / {cycle_distance})")
        cycles = round(distance / cycle_distance)
        steps  = (cycles * self.gait.loop.steps_in_gait)

        # 4. Calculate how long movement should be (distance / speed)
        movement_time = distance / speed  # seconds

        # 5. Calculate time per step to fit that timeframe (step_interval)
        step_interval    = movement_time / steps  # seconds / steps
        step_interval_ms = step_interval * 1_000 

        # print(f"[GE]  move() | {{Distance}} target: {distance}m, best: {cycles * cycle_distance}m ({cycles} * {cycle_distance}) | {{Speed}} {speed}m/s (step interval: {step_interval_ms}ms)")  # TODO: Remove, for debugging
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

        if self.gait.transition_in:
            self._instruction_queue.put( Instruction(self.gait.transition_in, step_interval_ms) )

        loop_instruction = Instruction(self.gait.loop, step_interval_ms, repeat_loop)
        self._instruction_queue.put( loop_instruction )

        if self.gait.transition_out:
            self._instruction_queue.put( Instruction(self.gait.transition_out, step_interval_ms) )
        
        print(f"[GE]  move({distance}m, {speed}m/s) requested")  # TODO: Remove, for debugging

    def move_direction(self, distance:float, bearing:float, speed:float):
        step_interval_ms, repeat_loop = self._calculate_move_distance_data(distance, speed)
        pass
    def move2(self, distance_x:float, distance_y:float, speed:float):
        pass



    # TODO: Later, dynamics model
    def _get_feedforward_torques(
        self,
        foot_positions: Point3DList,
        hip_origins: np.ndarray,
        target_motor_angles: list,
    ) -> np.ndarray:
        if not self.HEHE:
            return np.asarray( [0.0] *  JOINT_COUNT )  # TODO: Ignore Dynamics
        else:
            return self._dynamics.compute_support_torques(
                foot_positions,
                hip_origins,
                np.asarray(target_motor_angles, dtype=np.float64).reshape(JOINT_COUNT),
            )
    