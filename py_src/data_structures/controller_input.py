from typing import NamedTuple

JOYSTICK_DEADZONE:float = 0.05


# Adapted from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/controller.py
def apply_deadzone(value:float):
    """Zero out the value if it's too close to the center."""
    if abs(value) < JOYSTICK_DEADZONE:
        return 0.0
    return value

class JoyStickData(NamedTuple):
    delta_x:float
    delta_y:float

class Button():
    _was_down:bool = False

    def is_just_pressed(self, is_down:bool) -> bool:
        if is_down and ( not self._was_down ):
            self._was_down = True
            return True
        elif ( not is_down ) and self._was_down:
            self._was_down = False
        
        return False

class ControllerData():
    left_stick:JoyStickData
    right_stick:JoyStickData
    a_btn_down:bool
    b_btn_down:bool
    x_btn_down:bool
    y_btn_down:bool


    def __init__(self, left_stick:JoyStickData, right_stick:JoyStickData, a_btn_down:bool, b_btn_down:bool, x_btn_down:bool, y_btn_down:bool):
        self.left_stick  = left_stick
        self.right_stick = right_stick
        self.a_btn_down  = a_btn_down
        self.b_btn_down  = b_btn_down
        self.x_btn_down  = x_btn_down
        self.y_btn_down  = y_btn_down
