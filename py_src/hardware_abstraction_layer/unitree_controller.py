import struct

from data_structures.controller_input import ControllerData, JoyStickData, apply_deadzone
from hardware_abstraction_layer.input_layer import InputLayer

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


    class UnitreeController(InputLayer):
        """
        Handles Unitree Wireless Controller input via SDK2 DDS channel subscriber.
        Parses stick axes (-1.0 to 1.0) and bitmask button states.
        """

        # Button bitmask mappings for Unitree remote control
        BUTTON_MASKS = {
            'R1':         1 << 0,
            'L1':         1 << 1,
            'start':      1 << 2,
            'select':     1 << 3,
            'R2':         1 << 4,
            'L2':         1 << 5,
            'F1':         1 << 6,
            'F2':         1 << 7,
            'A':          1 << 8,
            'B':          1 << 9,
            'X':          1 << 10,
            'Y':          1 << 11,
            'dpad_up':    1 << 12,
            'dpad_right': 1 << 13,
            'dpad_down':  1 << 14,
            'dpad_left':  1 << 15,
        }

        def __init__(self, network_interface: str = "eth0", topic_name: str = "rt/lowstate"):
            """
            Initializes subscriber for Unitree remote controller topics.
            """
            self.latest = None  # (lx, ly, rx, ry, keys)

            ChannelFactoryInitialize(0, network_interface)

            # Subscribe to lowstate / wireless remote DDS topic
            self.subscriber = ChannelSubscriber(topic_name, LowState_)
            self.subscriber.Init(self._message_handler, 10)

            print(f"Using Unitree controller on '{network_interface}'")


        def _message_handler(self, msg: LowState_):
            data = bytes(msg.wireless_remote)
            keys = struct.unpack_from("<H", data, 2)[0]
            lx, rx, ry = struct.unpack_from("<fff", data, 4)
            ly = struct.unpack_from("<f", data, 20)[0]
            self.latest = (lx, ly, rx, ry, keys)

        def poll(self) -> ControllerData:
            """
            Polls latest remote status and returns normalized ControllerData.
            """
            if self.latest is None:
                lx = ly = rx = ry = 0.0
                keys = 0
            else:
                lx, ly, rx, ry, keys = self.latest

            return ControllerData(
                left_stick=JoyStickData(delta_x=apply_deadzone(lx), delta_y=apply_deadzone(ly)),
                right_stick=JoyStickData(delta_x=apply_deadzone(rx), delta_y=apply_deadzone(ry)),
                a_btn_down=bool(keys & self.BUTTON_MASKS['A']),
                b_btn_down=bool(keys & self.BUTTON_MASKS['B']),
                x_btn_down=bool(keys & self.BUTTON_MASKS['X']),
                y_btn_down=bool(keys & self.BUTTON_MASKS['Y']),
            )

except ImportError:
    pass
