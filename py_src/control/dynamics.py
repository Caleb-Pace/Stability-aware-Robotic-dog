"""Conservative stance support dynamics for the Unitree Go2.

This is intentionally a gravity/support model, not a full inverse-dynamics
controller.  It needs link lengths and masses, but not link inertias or
accelerometer state.  Swing legs are left to the position controller.
"""

import numpy as np
import numpy.typing as npt

from data_structures.constants import JOINT_COUNT, LEG_COUNT
from kinematics.fk_solver import calculate_joint_positions
from kinematics.ik_solver import (
    _HIP_ABDUCTOR_TORQUE_LIMIT,
    _HIP_TORQUE_LIMIT,
    _KNEE_TORQUE_LIMIT,
)


GRAVITY = 9.81
GO2_BASE_MASS = 6.921
GO2_HIP_MASS = 0.678
GO2_THIGH_MASS = 1.152
GO2_CALF_MASS = 0.241352
GO2_TOTAL_MASS = GO2_BASE_MASS + LEG_COUNT * (
    GO2_HIP_MASS + GO2_THIGH_MASS + GO2_CALF_MASS
)

_JOINT_TORQUE_LIMITS = np.tile(
    np.array(
        [
            _HIP_ABDUCTOR_TORQUE_LIMIT.maximum,
            _HIP_TORQUE_LIMIT.maximum,
            _KNEE_TORQUE_LIMIT.maximum,
        ],
        dtype=np.float64,
    ),
    LEG_COUNT,
)


class BasicDynamicsModel:
    """Generate bounded vertical support torque for known stance feet."""

    def __init__(
        self,
        support_scale: float = 0.35,
        swing_height_threshold: float = 0.02,
        max_torque_step: float = 1.0,
    ):
        if not 0.0 <= support_scale <= 1.0:
            raise ValueError("support_scale must be between 0 and 1")
        if swing_height_threshold < 0.0 or max_torque_step <= 0.0:
            raise ValueError("dynamics safety limits must be positive")

        self.support_scale = support_scale
        self.swing_height_threshold = swing_height_threshold
        self.max_torque_step = max_torque_step
        self._last_torques = np.zeros(JOINT_COUNT, dtype=np.float64)

    @staticmethod
    def _foot_jacobian(
        _origin: npt.NDArray[np.float64],
        angles: npt.NDArray[np.float64],
        is_left_leg: bool,
    ) -> npt.NDArray[np.float64]:
        """Return d(foot position)/d(joint angle) using the FK model."""
        jacobian = np.empty((3, 3), dtype=np.float64)
        epsilon = 1e-6
        for joint in range(3):
            plus = angles.copy()
            minus = angles.copy()
            plus[joint] += epsilon
            minus[joint] -= epsilon
            # FK accepts the body origin, while the gait engine stores the
            # hip origin separately.  Translation by that origin does not
            # affect a translational Jacobian.
            body_origin = np.zeros(3, dtype=np.float64)
            foot_plus = calculate_joint_positions(body_origin, plus, is_left_leg)[-1]
            foot_minus = calculate_joint_positions(body_origin, minus, is_left_leg)[-1]
            jacobian[:, joint] = (foot_plus - foot_minus) / (2.0 * epsilon)
        return jacobian

    def compute_support_torques(
        self,
        foot_positions: npt.NDArray[np.float64],
        hip_origins: npt.NDArray[np.float64],
        target_angles: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """Compute safe support torques for stance legs.

        A stance leg receives an equal share of the robot's weight.  A lifted
        leg receives no feed-forward torque, avoiding fighting the swing
        trajectory or driving a foot into an obstacle.
        """
        foot_positions = np.asarray(foot_positions, dtype=np.float64)
        hip_origins = np.asarray(hip_origins, dtype=np.float64)
        target_angles = np.asarray(target_angles, dtype=np.float64)
        if foot_positions.shape != (LEG_COUNT, 3):
            raise ValueError(f"foot_positions must have shape {(LEG_COUNT, 3)}")
        if hip_origins.shape != (LEG_COUNT, 3):
            raise ValueError(f"hip_origins must have shape {(LEG_COUNT, 3)}")
        if target_angles.shape != (JOINT_COUNT,):
            raise ValueError(f"target_angles must have shape {(JOINT_COUNT,)}")

        stance = foot_positions[:, 2] <= self.swing_height_threshold
        stance_count = int(np.count_nonzero(stance))
        desired = np.zeros(JOINT_COUNT, dtype=np.float64)
        if stance_count:
            force_z = GO2_TOTAL_MASS * GRAVITY / stance_count
            for leg in range(LEG_COUNT):
                if not stance[leg]:
                    continue
                joint_slice = slice(leg * 3, leg * 3 + 3)
                jacobian = self._foot_jacobian(
                    hip_origins[leg],
                    target_angles[joint_slice],
                    is_left_leg=(leg % 2 == 1),
                )
                desired[joint_slice] = jacobian.T @ np.array([0.0, 0.0, force_z])

        desired *= self.support_scale
        desired = np.clip(desired, -_JOINT_TORQUE_LIMITS, _JOINT_TORQUE_LIMITS)
        delta = np.clip(
            desired - self._last_torques,
            -self.max_torque_step,
            self.max_torque_step,
        )
        self._last_torques += delta
        return self._last_torques.copy()
