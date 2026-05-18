from abc import ABC, abstractmethod
from typing import List
import numpy as np

try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False

class RobotOutput(ABC):
    @abstractmethod
    def connect(self): pass

    @abstractmethod
    def send_commands(self, target_angles, torques): pass
    
    @abstractmethod
    def get_low_state(self): pass

class MujocoOutput(RobotOutput):
    def __init__(self, model, data, pids, ctrl_idx, qpos_idx):
        self.model = model
        self.data = data
        self.pids = pids
        self.ctrl_idx = ctrl_idx  # e.g., [0, 1, 2...]
        self.qpos_idx = qpos_idx  # Indices for joints in qpos
        self.dt = model.opt.timestep

    def connect(self):
        print("Connected to MuJoCo Simulator.")

    def get_low_state(self):
        # We mimic the Unitree LowState structure for the EKF
        class MockIMU:
            def __init__(self, acc): self.accelerometer = acc
        class MockState:
            def __init__(self, acc): self.imu_state = MockIMU(acc)
            self.wireless_remote = [0]*40 # No remote in sim (handled by pygame)

        # Pull Z-accel from MuJoCo (Assuming sensor named 'imu_acc')
        accel = self.data.sensor("imu_acc").data
        return MockState(accel)

    def send_commands(self, target_angles: List[float], feedforward_torques: List[float]):
        for leg in range(4):
            for j in range(3):
                global_idx = leg * 3 + j
                # Get current position from MuJoCo state
                current_p = self.data.qpos[self.qpos_idx[leg][j]]
                current_v = self.data.qvel[self.qpos_idx[leg][j] - 1] # qvel is offset by 1 due to freejoint

                # Calculate PID
                pid_t = self.pids[leg][j].update(target_angles[global_idx], current_p, current_v, self.dt)
                
                # Apply to simulation
                self.data.ctrl[self.ctrl_idx[leg][j]] = pid_t + feedforward_torques[global_idx]

class UnitreeGo2Output(RobotOutput):
    def __init__(self, network_interface="eth0"):
        self.interface = network_interface
        self.low_cmd = LowCmd_()
        self.low_state = LowState_()
        self.cmd_pub = None
        self.state_sub = None

    def _state_callback(self, msg: LowState_):
        self.low_state = msg

    def connect(self):
        ChannelFactoryInitialize(0, self.interface)
        # Publisher for commands
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        # Subscriber for state (for EKF and Remote)
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_callback, 10)

    def get_low_state(self):
        return self.low_state

    def send_commands(self, target_angles: List[float], feedforward_torques: List[float]):
        for i in range(12):
            m = self.low_cmd.motor_cmd[i]
            m.q = target_angles[i]
            m.dq = 0.0
            m.kp = 60.0 # Standard Go2 Stance Gain
            m.kd = 3.5
            m.tau = feedforward_torques[i]

        self.low_cmd.crc = CRC().Crc(self.low_cmd)
        self.cmd_pub.Write(self.low_cmd)