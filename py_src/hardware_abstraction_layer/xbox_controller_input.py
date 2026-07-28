import pygame
import sys

class GamepadController:
    """
    Handles Xbox/Playstation controller polling via Pygame.
    Includes deadzone filtering to prevent stick drift.
    """
    def __init__(self, deadzone=0.1):
        self.deadzone = deadzone
        
        # Initialize Pygame's joystick module
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            print("ERROR: No joystick detected. Please plug in your controller.")
            sys.exit(1)
            
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        print(f"Connected to: {self.joy.get_name()}")

    def _apply_deadzone(self, value):
        """Zero out the value if it's too close to the center."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def update(self):
        """
        Pumps the event queue and reads the latest axis values.
        Returns a dictionary of the sticks and important buttons.
        """
        # Pygame requires this to fetch the latest hardware events
        pygame.event.pump()

        # XBOX CONTROLLER AXIS MAPPING (Standard Pygame 2.x)
        # Note: Y-axes are usually inverted (Pushing UP gives a negative number). 
        # We negate them here so UP = Positive, DOWN = Negative.
        
        lx = self._apply_deadzone(self.joy.get_axis(0))
        ly = self._apply_deadzone(-self.joy.get_axis(1)) # Negated for standard math
        
        rx = self._apply_deadzone(self.joy.get_axis(3))
        ry = self._apply_deadzone(-self.joy.get_axis(4)) # Negated for standard math
        
        # Triggers usually go from -1.0 (unpressed) to 1.0 (fully pressed)
        l_trigger = self.joy.get_axis(2)
        r_trigger = self.joy.get_axis(5)
        
        # Buttons
        btn_a = self.joy.get_button(0)
        btn_start = self.joy.get_button(7)

        return {
            "lx": lx,           # Left Stick X (Strafe)
            "ly": ly,           # Left Stick Y (Forward/Back)
            "rx": rx,           # Right Stick X (Yaw/Turn)
            "ry": ry,           # Right Stick Y (Pitch)
            "l2": l_trigger,
            "r2": r_trigger,
            "a": btn_a,
            "start": btn_start
        }

# --- Standalone Test ---
if __name__ == "__main__":
    import time
    pad = GamepadController()
    print("Polling controller... Press CTRL+C to stop.")
    try:
        while True:
            inputs = pad.update()
            # Only print if we are actually moving a stick
            if inputs["lx"] != 0 or inputs["ly"] != 0 or inputs["rx"] != 0:
                print(f"LX: {inputs['lx']:.2f} | LY: {inputs['ly']:.2f} | RX: {inputs['rx']:.2f} | RY: {inputs['ry']:.2f}")
            # 20Hz just for this test print
            if inputs["l2"] != 0 or inputs["r2"] != 0:
                print(f"L2: {inputs['l2']:.2f} | R2: {inputs['r2']:.2f}")
            if inputs['a']:
                print("Button A pressed!")
            if inputs['start']:
                print("Start button pressed!")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Test ended.")
        pygame.quit()