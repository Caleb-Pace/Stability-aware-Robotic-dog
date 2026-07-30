from abc import ABC, abstractmethod
from typing import List
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, max_torque):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_torque = max_torque
        self.integral = 0.0

    def update(self, target, current, velocity, dt):
        error = target - current
        self.integral += error * dt

        output = (self.kp * error) + (self.ki * self.integral) - (self.kd * velocity)
        
        return np.clip(output, -self.max_torque, self.max_torque)


class RobotOutput(ABC):
    @abstractmethod
    def connect(self): pass

    @abstractmethod
    def send_commands(self, target_angles: List[float], feedforward_torques: List[float]): pass
    
    @abstractmethod
    def get_low_state(self): pass


class MujocoOutput(RobotOutput):
    """
    MuJoCo output layer with internal PID joint control and state mocking for EKF.
    """
    def __init__(self, model, data, pids, ctrl_idx=None, qpos_idx=None):
        self.model = model
        self.data = data
        self.dt = model.opt.timestep

        # --- 2. Convert the PIDs dictionary into actual PID Controller objects ---
        self.pid_controllers = []
        if isinstance(pids, dict):
            for leg in range(4):
                leg_pids = []
                for j in range(3):  # 3 joints per leg
                    pid = PIDController(
                        kp=pids["kp"][j],
                        ki=pids["ki"][j],
                        kd=pids["kd"][j],
                        max_torque=pids["torque_max"][j]
                    )
                    leg_pids.append(pid)
                self.pid_controllers.append(leg_pids)
        else:
            self.pid_controllers = pids  # Fallback if already instantiated

        # Default indices for a standard 12-DOF quadruped (4 legs x 3 joints)
        self.ctrl_idx = ctrl_idx if ctrl_idx is not None else [
            [0, 1, 2],     # Front Left (Hip, Thigh, Calf)
            [3, 4, 5],     # Front Right
            [6, 7, 8],     # Rear Left
            [9, 10, 11]    # Rear Right
        ]
        
        # Default qpos joint indices (skipping 7 freejoint qpos indices: x,y,z + quat)
        self.qpos_idx = qpos_idx if qpos_idx is not None else [
            [7, 8, 9],     # Front Left
            [10, 11, 12],  # Front Right
            [13, 14, 15],  # Rear Left
            [16, 17, 18]   # Rear Right
        ]

        self.has_freejoint = self.model.nv > self.model.nu
        self.qvel_offset = 1 if self.has_freejoint else 0

    def connect(self):
        print("[MujocoOutput] Connected to MuJoCo Simulator.")

    def get_low_state(self):
        class MockIMU:
            def __init__(self, acc): 
                self.accelerometer = acc

        class MockState:
            def __init__(self, acc): 
                self.imu_state = MockIMU(acc)
                self.wireless_remote = [0] * 40  # Remote buffer placeholder

        try:
            accel = np.copy(self.data.sensor("imu_acc").data)
        except KeyError:
            accel = np.array([0.0, 0.0, 9.81])

        return MockState(accel)

    def send_commands(self, target_angles: List[float], feedforward_torques: List[float]):
        """
        Calculates PID control signals and writes total torque to self.data.ctrl.
        """
        target_angles = np.asarray(target_angles).flatten()
        feedforward_torques = np.asarray(feedforward_torques).flatten() if feedforward_torques is not None else np.zeros(12)

        for leg in range(4):
            for j in range(3):
                global_idx = leg * 3 + j
                qp_idx = self.qpos_idx[leg][j]
                qv_idx = qp_idx - self.qvel_offset

                # Fetch current joint angular state from MuJoCo
                current_p = self.data.qpos[qp_idx]
                current_v = self.data.qvel[qv_idx]

                # --- 3. Use the instantiated PID controllers ---
                pid_t = self.pid_controllers[leg][j].update(
                    target_angles[global_idx], 
                    current_p, 
                    current_v, 
                    self.dt
                )
                
                # Apply computed control torque to actuator target
                ctrl_i = self.ctrl_idx[leg][j]
                self.data.ctrl[ctrl_i] = pid_t + feedforward_torques[global_idx]