from data_structures.controller_input import ControllerData, JoyStickData, apply_deadzone
from hardware_abstraction_layer.input_layer import InputLayer

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_


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

        def __init__(self, network_interface: str = "eth0", topic_name: str = "rt/wirelessremote"):
            """
            Initializes subscriber for Unitree remote controller topics.
            """
            self.latest_msg = None

            ChannelFactoryInitialize(0, network_interface)

            # Subscribe to lowstate / wireless remote DDS topic
            self.subscriber = ChannelSubscriber(topic_name, WirelessController_)
            self.subscriber.Init(self._message_handler, 10)

        def _message_handler(self, msg: WirelessController_):
            self.latest_msg = msg

        def poll(self) -> ControllerData:
            """
            Polls latest remote status and returns normalized ControllerData.
            """
            if self.latest_msg is None:
                return ControllerData(
                    left_stick=JoyStickData(delta_x=0.0, delta_y=0.0),
                    right_stick=JoyStickData(delta_x=0.0, delta_y=0.0),
                    a_btn_down=False,
                    b_btn_down=False,
                    x_btn_down=False,
                    y_btn_down=False,
                )

            lx = apply_deadzone(float(getattr(self.latest_msg, 'lx', 0.0)))
            ly = apply_deadzone(float(getattr(self.latest_msg, 'ly', 0.0)))
            rx = apply_deadzone(float(getattr(self.latest_msg, 'rx', 0.0)))
            ry = apply_deadzone(float(getattr(self.latest_msg, 'ry', 0.0)))
            keys = int(getattr(self.latest_msg, 'keys', 0))

            # Map joystick axes into JoyStickData structures
            left_stick = JoyStickData(delta_x=lx, delta_y=ly)
            right_stick = JoyStickData(delta_x=rx, delta_y=ry)

            # Extract button states using bitmasks
            a_btn_down = bool(keys & self.BUTTON_MASKS['A'])
            b_btn_down = bool(keys & self.BUTTON_MASKS['B'])
            x_btn_down = bool(keys & self.BUTTON_MASKS['X'])
            y_btn_down = bool(keys & self.BUTTON_MASKS['Y'])

            return ControllerData(
                left_stick=left_stick,
                right_stick=right_stick,
                a_btn_down=a_btn_down,
                b_btn_down=b_btn_down,
                x_btn_down=x_btn_down,
                y_btn_down=y_btn_down,
            )

except ImportError:
    pass
