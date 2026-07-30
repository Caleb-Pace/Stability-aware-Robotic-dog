from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1' 
import pygame
import sys

from data_structures.controller_input import ControllerData, JoyStickData
from hardware_abstraction_layer.input_layer import InputLayer

JOYSTICK_DEADZONE: float = 0.12
AXIS_LX, AXIS_LY, AXIS_RX = 0, 1, 3
BTN_A, BTN_Y = 0, 3  # Standard Xbox mapping

class GamepadController(InputLayer):
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("ERROR: No controller detected.")
            sys.exit(1)

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"Using \"{self.controller.get_name()}\" controller")

    def _apply_deadzone(self, value):
        if abs(value) < JOYSTICK_DEADZONE: return 0.0
        return value

    def poll(self) -> ControllerData:
        pygame.event.pump() 

        lx = self._apply_deadzone(self.controller.get_axis(AXIS_LX))
        ly = self._apply_deadzone(self.controller.get_axis(AXIS_LY))
        rx = self._apply_deadzone(self.controller.get_axis(AXIS_RX))

        btn_a = bool(self.controller.get_button(BTN_A))
        btn_y = bool(self.controller.get_button(BTN_Y))

        return ControllerData(
            left_stick=JoyStickData(delta_x=lx, delta_y=ly),
            right_stick=JoyStickData(delta_x=rx, delta_y=0.0),
            btn_a=btn_a, 
            btn_y=btn_y
        )