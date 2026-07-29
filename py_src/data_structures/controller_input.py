from typing import NamedTuple

class JoyStickData(NamedTuple):
    delta_x:float
    delta_y:float

class ControllerData():
    left_stick:JoyStickData
    right_stick:JoyStickData

    def __init__(self, left_stick:JoyStickData, right_stick:JoyStickData):
        self.left_stick = left_stick
        self.right_stick = right_stick
