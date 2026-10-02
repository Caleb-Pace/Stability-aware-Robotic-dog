import argparse
import threading
import time
from collections.abc import Callable
from typing import Any

from hardware_abstraction_layer.robot_output import UnitreeGo2Output

try:
    from .stability_ekf import UnitreeGo2StabilityEKF
except ImportError:
    from stability_ekf import UnitreeGo2StabilityEKF


class Go2LowLevelRecoveryNode:
    """Low-level recovery monitor using the project's hardware output layer.

    The output layer owns the low-state subscription and low-command publisher.
    Recovery action construction is deliberately injectable until the action
    system is implemented.
    """

    def __init__(
        self,
        network_interface: str,
        action_builder: Callable[[Any], Any | None] | None = None,
    ):
        self.ekf = UnitreeGo2StabilityEKF(dt=0.002)
        self.output = UnitreeGo2Output(network_interface=network_interface)
        self.action_builder = action_builder
        self.stop_event = threading.Event()
        self.output_thread = threading.Thread(
            target=self.output.connect,
            args=(self.stop_event,),
            name="go2-low-level-output",
        )

    def run(self) -> None:
        self.output_thread.start()
        start_time = time.monotonic()
        last_print_time = start_time

        print("Low-level recovery node started.")
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                estimate = self.ekf.step_low_state(self.output.get_low_state())

                if now - last_print_time >= 0.1:
                    last_print_time = now
                    flag = "[OK]" if not estimate.is_tipping else "[ALERT]"
                    print(
                        f"{flag} t={now - start_time:6.2f}s | "
                        f"Status: {estimate.status.name:<13} | "
                        f"Roll: {estimate.roll_deg:5.1f} deg | "
                        f"Pitch: {estimate.pitch_deg:5.1f} deg | "
                        f"Contacts: {estimate.contact_count}/4"
                    )

                if estimate.is_tipping:
                    self._trigger_low_level_recovery(estimate)

                self.stop_event.wait(self.ekf.dt)
        finally:
            self.stop()

    def _trigger_low_level_recovery(self, estimate: Any) -> None:
        """Build and publish a low-level Action when the action system is ready."""
        if self.action_builder is None:
            return

        action = self.action_builder(estimate)
        if action is not None:
            self.output.send_action(action)

    def stop(self) -> None:
        self.stop_event.set()
        if self.output_thread.is_alive() and threading.current_thread() is not self.output_thread:
            self.output_thread.join(timeout=2.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Go2 low-level recovery monitor")
    parser.add_argument("--interface", default="eth0", help="Network interface used by the robot backend")
    args = parser.parse_args()

    node = Go2LowLevelRecoveryNode(args.interface)
    try:
        node.run()
    except KeyboardInterrupt:
        print("\nStopping low-level recovery node...")
        node.stop()


if __name__ == "__main__":
    main()