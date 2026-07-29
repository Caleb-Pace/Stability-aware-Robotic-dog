import numpy as np
from kinematic_controller.output import RobotOutput


class DummyOutput(RobotOutput):
    def connect(self):
        print("Dummy Output: connected!")
        pass

    def send_commands(self, target_angles, torques):
        if len(target_angles) < 12:
            print(f"ERROR: not enough angles ({len(target_angles)} == 12)(from Dummy Output)")
            return
        if len(target_angles) > 12:
            print(f"ERROR: too many angles ({len(target_angles)} == 12)(from Dummy Output)")
            return
        if len(target_angles) != len(torques):
            print("ERROR: angle-torque mismatch (from Dummy Output)")
            return

        print(f"Dummy Output - Recieved Instructions:")
        leg_num:int = 0
        for angle, torque in zip(target_angles, torques):
            print(f"  Leg[{leg_num}] {{angle target: {np.round(np.degrees(angle), 3):>6}° }}; {{torque: {np.round(torque, 5):>8} Nm }}")
            leg_num += 1
    
    def get_low_state(self):
        print("Dummy Output: Low level state requested!")
        pass