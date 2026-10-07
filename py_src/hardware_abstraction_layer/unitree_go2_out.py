import time
import threading

from data_structures import Position
from hardware_abstraction_layer import OutputLayer

try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    UNITREE_SDK_AVAILABLE = True


    # Level Flags
    _HIGHLEVEL_FLAG    = 0xEE
    _LOWLEVEL_FLAG     = 0xFF
    _TRIGERLEVEL_FLAG  = 0xF0

    _GO2_PosStopF = 2.146e9
    _GO2_VelStopF = 16000.0


    # TODO: Identify how PID is meant to be used here
    # TODO: Later, `self.low_state.motor_state[i].q` is how you get current position
    class UnitreeGo2(OutputLayer):
        def __init__(self, network_interface:str = "eth0"):  # Default to loopback interface (run locally)
            self.interface = network_interface
            self.crc = CRC()

            self.low_cmd   = unitree_go_msg_dds__LowCmd_()
            self.low_state = unitree_go_msg_dds__LowState_()  # TODO: diff, = None

            self.lowcmd_pub   = None
            self.lowstate_sub = None

            self.ready = threading.Event()

        def _init_lowcmd(self):
            self.low_cmd.head[0]    = 0xFE
            self.low_cmd.head[1]    = 0xEF
            self.low_cmd.level_flag = _LOWLEVEL_FLAG
            self.low_cmd.gpio       = 0

            # TODO: Comment, why 20
            for i in range(20):
                self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode  # TODO: Comment, what is this number, and what is PMSM
                self.low_cmd.motor_cmd[i].q    = _GO2_PosStopF
                self.low_cmd.motor_cmd[i].kp   = 0
                self.low_cmd.motor_cmd[i].dq   = _GO2_VelStopF
                self.low_cmd.motor_cmd[i].kd   = 0
                self.low_cmd.motor_cmd[i].tau  = 0

        def _lowstate_message_handler(self, msg:LowState_):
            self.low_state = msg

        def connect(self, terminate_connection:threading.Event):
            try:
                ChannelFactoryInitialize(0, self.interface)

                # Publisher for commands
                self._init_lowcmd()
                self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
                self.lowcmd_pub.Init()

                # Subscriber for state (for EKF and Remote)
                self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
                self.lowstate_sub.Init(self._lowstate_message_handler, 10)

                # Release motor control
                self.sc = SportClient()  
                self.sc.SetTimeout(5.0)
                self.sc.Init()

                self.msc = MotionSwitcherClient()
                self.msc.SetTimeout(5.0)
                self.msc.Init()

                _, result = self.msc.CheckMode()
                while result['name']:  # Wait for motor release
                    self.sc.StandDown() # Sit
                    self.msc.ReleaseMode()
                    _, result = self.msc.CheckMode()
                    time.sleep(1)


                # Connected!
                self.ready.set()

                # Wait until termination event is triggered
                terminate_connection.wait()

            finally:
                # Start clean up!
                self.ready.clear()

                # Clean up subscriber and publisher connections on termination
                if self.lowstate_sub is not None:
                    if hasattr(self.lowstate_sub, "Close"):
                        self.lowstate_sub.Close()
                    self.lowstate_sub = None

                if self.lowcmd_pub is not None:
                    if hasattr(self.lowcmd_pub, "Close"):
                        self.lowcmd_pub.Close()
                    self.lowcmd_pub = None

                # Re-enable the built-in controller
                self.msc.SelectMode("normal")
                time.sleep(2)

        def get_low_state(self):
            return self.low_state

        def send_position(self, position:Position):
            if not self.ready.is_set():
                return

            for i in range(12):
                m = self.low_cmd.motor_cmd[i]

                m.q    = position.target_angles[i]
                m.dq   = 0
                m.kp   = 60.0 # Standard Go2 Stance Gain
                m.kd   = 5.0
                m.tau  = 0  # TODO: Remove, temporary to match example
                # m.tau  = position.feedforward_torques[i]  # TODO: Uncomment later when undestood

            # Calculate CRC on the populated command structure
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_pub.Write(self.low_cmd)

except ImportError:
    UNITREE_SDK_AVAILABLE = False
