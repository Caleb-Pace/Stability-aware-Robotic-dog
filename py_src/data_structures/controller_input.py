from typing import NamedTuple

class JoyStickData:
    def __init__(self, delta_x: float, delta_y: float):
        self.delta_x = delta_x
        self.delta_y = delta_y

class ControllerData:
    def __init__(self, left_stick: JoyStickData, right_stick: JoyStickData, btn_a: bool = False, btn_y: bool = False):
        self.left_stick = left_stick
        self.right_stick = right_stick
        self.btn_a = btn_a
        self.btn_y = btn_y