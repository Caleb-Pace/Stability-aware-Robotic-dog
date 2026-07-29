from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'  # Silence hello from pygame
import pygame

import sys
from data_structures.controller_input import ControllerData, JoyStickData
from hardware_abstraction_layer.input_layer import InputLayer


JOYSTICK_DEADZONE:float = 0.05

# Adapted from: https://github.com/maanas444/go2-simulation/blob/main/unitreego2/controller.py
class GamepadController(InputLayer):
    def __init__(self):
        """
        Handles Xbox/Playstation controller polling via Pygame.
        Includes deadzone filtering to prevent stick drift.
        """

        # Initialize Pygame's joystick module
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("ERROR: No controller detected. Please plug in your controller.")
            sys.exit(1)

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

        print(f"Using \"{self.controller.get_name()}\" controller")


    def _apply_deadzone(self, value):
        """Zero out the value if it's too close to the center."""
        if abs(value) < JOYSTICK_DEADZONE:
            return 0.0
        return value

    def poll(self) -> ControllerData:
        pygame.event.pump()  # Fetch latest controller state

        # Read joy sticks        
        lx = self._apply_deadzone(self.controller.get_axis(0))
        ly = self._apply_deadzone(-self.controller.get_axis(1))
        left_stick:JoyStickData = JoyStickData(delta_x=lx, delta_y=ly)

        rx = self._apply_deadzone(self.controller.get_axis(2))
        ry = self._apply_deadzone(-self.controller.get_axis(3))
        right_stick:JoyStickData = JoyStickData(delta_x=rx, delta_y=ry)


        return ControllerData(left_stick, right_stick)