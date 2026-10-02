from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'  # Silence hello from pygame
import pygame

import sys
from data_structures.controller_input import ControllerData, JoyStickData, apply_deadzone
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

        # Read joy sticks
        lx = apply_deadzone(self.controller.get_axis(0))
        ly = apply_deadzone(-self.controller.get_axis(1))
        left_stick:JoyStickData = JoyStickData(delta_x=lx, delta_y=ly)

        if self.controller.get_axis(4) == -1.0:  # Using right joysticks as 2 & 3
            rx = apply_deadzone(self.controller.get_axis(2))
            ry = apply_deadzone(-self.controller.get_axis(3))
        else:                                    # Using right joysticks as 3 & 4
            rx = apply_deadzone(self.controller.get_axis(3))
            ry = apply_deadzone(-self.controller.get_axis(4))
        right_stick:JoyStickData = JoyStickData(delta_x=rx, delta_y=ry)

        # Read buttons
        a_btn_down = self.controller.get_button(0)
        b_btn_down = self.controller.get_button(1)
        x_btn_down = self.controller.get_button(2)
        y_btn_down = self.controller.get_button(3)

        return ControllerData(
            left_stick=left_stick,
            right_stick=right_stick,
            a_btn_down=a_btn_down,
            b_btn_down=b_btn_down,
            x_btn_down=x_btn_down,
            y_btn_down=y_btn_down,
        )
