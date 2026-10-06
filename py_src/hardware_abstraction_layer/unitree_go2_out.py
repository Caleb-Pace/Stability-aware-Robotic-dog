import threading

from data_structures import Position
from hardware_abstraction_layer import OutputLayer

try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
    from unitree_sdk2py.utils.crc import CRC

    UNITREE_SDK_AVAILABLE = True


    # TODO: Identify how PID is meant to be used here
    class UnitreeGo2(OutputLayer):
        def __init__(self, network_interface:str = "eth0"):  # Default to loopback interface (run locally)
            self.interface = network_interface
            self.low_cmd = unitree_go_msg_dds__LowCmd_()
            self.low_state = unitree_go_msg_dds__LowState_()
            self.cmd_pub = None
            self.state_sub = None

        def _state_callback(self, msg:LowState_):
            self.low_state = msg

        def connect(self, terminate_connection:threading.Event):
            ChannelFactoryInitialize(0, self.interface)

            # Publisher for commands
            self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self.cmd_pub.Init()

            # Subscriber for state (for EKF and Remote)
            self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self.state_sub.Init(self._state_callback, 10)

            # Wait until termination event is triggered
            terminate_connection.wait()

            # Clean up subscriber and publisher connections on termination
            if self.state_sub is not None:
                if hasattr(self.state_sub, "Close"):
                    self.state_sub.Close()
                self.state_sub = None

            if self.cmd_pub is not None:
                if hasattr(self.cmd_pub, "Close"):
                    self.cmd_pub.Close()
                self.cmd_pub = None

        def get_low_state(self):
            return self.low_state

        def send_position(self, position:Position):
            # Set protocol framing and control level flags
            self.low_cmd.head[0] = 0xFE
            self.low_cmd.head[1] = 0xEF
            self.low_cmd.level_flag = 0xFF
            self.low_cmd.gpio = 0

            for i in range(12):
                m = self.low_cmd.motor_cmd[i]
                m.mode = 0x01  # Enable motor control mode (0x01 = Servo mode)
                m.q = position.target_angles[i]
                m.dq = 0.0
                m.kp = 60.0 # Standard Go2 Stance Gain
                m.kd = 3.5
                m.tau = position.feedforward_torques[i]

            # Calculate CRC on the populated command structure
            self.low_cmd.crc = CRC().Crc(self.low_cmd)
            self.cmd_pub.Write(self.low_cmd)

except ImportError:
    UNITREE_SDK_AVAILABLE = False
