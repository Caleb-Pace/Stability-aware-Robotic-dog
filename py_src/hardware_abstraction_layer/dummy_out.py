import threading
import numpy as np

from data_structures import Position
from hardware_abstraction_layer.output_layer import OutputLayer


class DummyOutput(OutputLayer):
    _last_angle_set = None

    def connect(self, terminate_connection:threading.Event):
        print("[DO]  Dummy Output: connected!")

        while not terminate_connection.is_set():
            pass
        print("[DO]  Dummy Output: connection terminated!")

    def send_position(self, position:Position):
        target_angles       = position.target_angles
        feedforward_torques = position.feedforward_torques
        # TODO: Uncomment, for debugging
        # if (self._last_angle_set is not None) and np.array_equal(target_angles, self._last_angle_set):
        #     return  # Don't send duplicate debug messages
        # self._last_angle_set = target_angles

        if len(target_angles) != len(feedforward_torques):
            print("[DO]  ERROR: angle-torque mismatch (from Dummy Output)")
            return

        print("[DO]  Dummy Output - Recieved Instructions:")
        print(f"    | Leg # |       Abd motor       |       Hip motor       |       Knee motor      |")
        print(f"    | ----- | --------------------- | --------------------- | --------------------- |")
        leg_num:int = 0
        for angle, torque in zip(target_angles, feedforward_torques):

            angle_str  = f"{np.round(np.degrees(angle), 3):>7}°"
            torque_str = f"{np.round(torque, 5):>8} Nm"

            if leg_num % 3 == 0:
                print(f"    | {((leg_num // 3) + 1):>5} | ", end="")
            print(f"{angle_str:>8}; {torque_str:<11} | ", end="")
            if leg_num % 3 == 2:
                print()

            leg_num += 1
        print()

    # TODO: Implement
    def get_low_state(self):
        print("[DO]  Dummy Output: Low level state requested!")
        pass