from typing import NamedTuple
from pygame.joystick import JoystickType

JOYSTICK_DEADZONE:float = 0.05

class JoyStickData(NamedTuple):
    delta_x:float
    delta_y:float

class ControllerData():
    left_stick:JoyStickData
    right_stick:JoyStickData
    a_btn_down:bool
    b_btn_down:bool
    x_btn_down:bool
    y_btn_down:bool


    # Adapted from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/controller.py
    def _apply_deadzone(self, value):
        """Zero out the value if it's too close to the center."""
        if abs(value) < JOYSTICK_DEADZONE:
            return 0.0
        return value

    def __init__(self, controller:JoystickType):
        # Read joy sticks
        lx = self._apply_deadzone(controller.get_axis(0))
        ly = self._apply_deadzone(-controller.get_axis(1))
        self.left_stick:JoyStickData = JoyStickData(delta_x=lx, delta_y=ly)

        if controller.get_axis(4) == -1.0:  # Using right joysticks as 2 & 3
            rx = self._apply_deadzone(controller.get_axis(2))
            ry = self._apply_deadzone(-controller.get_axis(3))
        else:                                    # Using right joysticks as 3 & 4
            rx = self._apply_deadzone(controller.get_axis(3))
            ry = self._apply_deadzone(-controller.get_axis(4))
        self.right_stick:JoyStickData = JoyStickData(delta_x=rx, delta_y=ry)

        # Read buttons
        self.a_btn_down = controller.get_button(0)
        self.b_btn_down = controller.get_button(1)
        self.x_btn_down = controller.get_button(2)
        self.y_btn_down = controller.get_button(3)
