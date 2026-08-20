
# filepath: /home/maanas/Desktop/Projects/Stability-aware-Robotic-dog/py_src/hardware_abstraction_layer/robot_out.py
import inspect
import threading
from types import SimpleNamespace

from typing import Any

import numpy as np

from data_structures import Action
from hardware_abstraction_layer.output_layer import OutputLayer

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    ChannelFactoryInitialize = None
    ChannelPublisher = None
    ChannelSubscriber = None
    LowCmd_ = None
    LowState_ = None
    CRC = None
    UNITREE_SDK_AVAILABLE = False


def _fallback_low_state():
    return SimpleNamespace(
        imu_state=SimpleNamespace(accelerometer=np.array([0.0, 0.0, 9.81], dtype=np.float64)),
        wireless_remote=[0] * 40,
    )


def _fallback_motor_cmd():
    return SimpleNamespace(q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)


def _make_low_cmd():
    if not UNITREE_SDK_AVAILABLE:
        return None

    try:
        return LowCmd_()
    except TypeError:
        sig = inspect.signature(LowCmd_)
        params = list(sig.parameters.keys())

        values = {
            "head": 0,
            "level_flag": 0,
            "frame_reserve": 0,
            "sn": [0] * 8,
            "version": 0,
            "bandwidth": 0,
            "motor_cmd": [_fallback_motor_cmd() for _ in range(12)],
            "bms_cmd": None,
            "wireless_remote": [0] * 40,
            "led": None,
            "fan": None,
            "gpio": None,
            "reserve": 0,
            "crc": 0,
        }

        args = [values.get(name, 0) for name in params]
        return LowCmd_(*args)


class UnitreeGo2Output(OutputLayer):
    def __init__(self, network_interface: str = "eth0", command_rate_hz: float = 200.0):
        self.network_interface = network_interface
        self.command_period = 1.0 / command_rate_hz if command_rate_hz > 0 else 0.005

        self.low_cmd = _make_low_cmd()
        self.low_state = _fallback_low_state()

        self.cmd_pub = None
        self.state_sub = None
        self._connected = False
        self._latest_action: Action | None = None
        self._lock = threading.Lock()

    def _state_callback(self, msg: Any):
        self.low_state = msg

    def _publish_action(self, action: Action):
        if not self._connected or self.cmd_pub is None or self.low_cmd is None:
            return

        target_angles = np.asarray(action.target_angles, dtype=np.float64).flatten()
        feedforward_torques = np.asarray(action.feedforward_torques, dtype=np.float64).flatten()

        if target_angles.size != 12 or feedforward_torques.size != 12:
            raise ValueError("UnitreeGo2Output expects 12 joint targets and 12 feedforward torques")

        for i in range(12):
            motor = self.low_cmd.motor_cmd[i]
            motor.q = float(target_angles[i])
            motor.dq = 0.0
            motor.kp = 60.0
            motor.kd = 3.5
            motor.tau = float(feedforward_torques[i])

        if CRC is not None:
            self.low_cmd.crc = CRC().Crc(self.low_cmd)

        self.cmd_pub.Write(self.low_cmd)

    def connect(self, terminate_connection: threading.Event):
        if not UNITREE_SDK_AVAILABLE:
            raise RuntimeError("unitree_sdk2py is not installed")

        ChannelFactoryInitialize(0, self.network_interface)

        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()

        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._state_callback, 10)

        self._connected = True
        print(f"[RO]  Connected to Unitree Go2 on {self.network_interface}")

        while not terminate_connection.is_set():
            with self._lock:
                latest_action = self._latest_action
            if latest_action is not None:
                self._publish_action(latest_action)
            terminate_connection.wait(self.command_period)

        self._connected = False
        print("[RO]  Unitree Go2 connection terminated")

    def send_action(self, action: Action):
        with self._lock:
            self._latest_action = action
        self._publish_action(action)

    def send_stand_position(self):
        self.send_action(Action(target_angles=np.zeros(12), feedforward_torques=np.zeros(12)))

    def get_low_state(self):
        return self.low_state