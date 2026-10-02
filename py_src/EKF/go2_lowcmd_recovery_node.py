import argparse
import threading
import time
from collections.abc import Callable
from typing import Any

from control.gait_engine import GaitEngine
from control.actions import STAND
from data_structures.leg_trajectories import LegTrajectories
from hardware_abstraction_layer.output_layer import OutputLayer

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
        output: OutputLayer,
        engine: GaitEngine | None = None,
        action_builder: Callable[[Any], LegTrajectories | None] | None = None,
        manage_output: bool = True,
    ):
        self.ekf = UnitreeGo2StabilityEKF(dt=0.002)
        self.output = output
        self.engine = engine
        self.action_builder = action_builder or (lambda _estimate: STAND)
        self.manage_output = manage_output
        self._recovery_requested = False
        self.stop_event = threading.Event()
        self.output_thread = threading.Thread(
            target=self.output.connect,
            args=(self.stop_event,),
            name="go2-low-level-output",
        )

    def run(self) -> None:
        if self.manage_output:
            self.output_thread.start()
        start_time = time.monotonic()
        last_print_time = start_time

        print("Low-level recovery node started.")
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                low_state = self.output.get_low_state()
                if low_state is None:
                    self.stop_event.wait(self.ekf.dt)
                    continue
                estimate = self.ekf.step_low_state(low_state)

                if self.engine is not None and estimate.is_tipping and not self._recovery_requested:
                    self._trigger_low_level_recovery(estimate)
                    self._recovery_requested = True
                elif self.engine is not None and not estimate.is_tipping:
                    self._recovery_requested = False

                if now - last_print_time >= 0.1:
                    last_print_time = now
                    flag = "[OK]" if not estimate.is_tipping else "[ALERT]"
                    print(
                        f"{flag} t={now - start_time:6.2f}s | "
                        f"Status: {estimate.status.name:<13} | "
                        f"Roll: {estimate.roll_deg:5.1f} deg | "
                        f"Pitch: {estimate.pitch_deg:5.1f} deg | "
                        f"Contacts: {self._format_contacts(estimate)}"
                    )

                self.stop_event.wait(self.ekf.dt)
        finally:
            self.stop()

    def _trigger_low_level_recovery(self, estimate: Any) -> None:
        """Queue one recovery action through the gait engine."""
        action = self.action_builder(estimate)
        if action is not None:
            self.engine.perform_action(action)

    @staticmethod
    def _format_contacts(estimate: Any) -> str:
        return " ".join(
            f"{name}:{'Y' if is_contact else 'N'}"
            for name, is_contact in zip(("FL", "FR", "RL", "RR"), estimate.foot_contacts)
        )

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