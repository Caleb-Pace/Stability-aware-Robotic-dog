import numpy as np
from kinematic_controller.output import RobotOutput


class DummyOutput(RobotOutput):
    _last_angle_set = None
    
    def connect(self):
        print("Dummy Output: connected!")
        pass

    def send_commands(self, target_angles, torques):
        if (self._last_angle_set is not None) and np.array_equal(target_angles, self._last_angle_set):
            return  # Don't send duplicate debug messages
        self._last_angle_set = target_angles

        if len(target_angles) < 4:
            print(f"ERROR: not enough leg angles ({len(target_angles)} == 4)(from Dummy Output)")
            return
        if len(target_angles) > 4:
            print(f"ERROR: too many leg angles ({len(target_angles)} == 4)(from Dummy Output)")
            return
        if len(target_angles.ravel()) != len(torques):
            print("ERROR: angle-torque mismatch (from Dummy Output)")
            return

        print(f"Dummy Output - Recieved Instructions:")
        leg_num:int = 0
        for angle, torque in zip(target_angles.ravel(), torques):
            if leg_num % 3 == 0:
                print(f"  Leg[{(leg_num // 3)}]:")

            print(f"         Motor[{leg_num}] {{angle target: {np.round(np.degrees(angle), 3):>7}° }}; {{torque: {np.round(torque, 5):>8} Nm }}")

            leg_num += 1
    
    def get_low_state(self):
        print("Dummy Output: Low level state requested!")
        pass