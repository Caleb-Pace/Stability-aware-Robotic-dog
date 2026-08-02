from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'  # Silence hello from pygame
import pygame

import sys
from data_structures.controller_input import ControllerData, JoyStickData
from hardware_abstraction_layer.input_layer import InputLayer


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

    def poll(self) -> ControllerData:
        pygame.event.pump()  # Fetch latest controller state

        return ControllerData(self.controller)