import numpy as np
from data_structures import Point3D, Vector
from typing import Tuple


# Zero offsets for angles #
#     Based on: https://support.unitree.com/home/en/developer
_ANGLE_ZERO_OFFSETS = np.array([
     np.pi/2,  # Abductor 90 deg
    -np.pi/2,  # Hip     -90 deg
     0,        # Knee      0 deg
])  # In Radians

# Link Lengths in meters #
_HIP_OFFSET   = 0.01  # TODO: Placeholder, find real value  # CANNOT BE ZERO
_THIGH_LENGTH = 0.213
_CALF_LENGTH  = 0.213

# Rotation limits (min, max) in Radians #
_HIP_ABDUCTOR_ROT_RANGE = (-1.0472,  1.0472)   # approx. -60  to  60  deg
_FRONT_HIP_ROT_RANGE    = (-1.5708,  3.4907)   # approx. -90  to  200 deg
_BACK_HIP_ROT_RANGE     = (-0.5236,  4.5379)   # approx. -30  to  260 deg
_KNEE_ROT_RANGE         = (-2.7227, -0.83776)  # approx. -155 to -48  deg

# Output torque limits in Newton-meters #
_KNEE_TORQUE_LIMIT = (-45.43, 45.43)

# Reachability limits #
_MAX_RANGE_LENGTH = _THIGH_LENGTH + (_CALF_LENGTH * _KNEE_ROT_RANGE[1])


def get_unit_vectors_of_a_plane(normal_vector:Vector) -> Tuple[Vector, Vector]:
    # Normalise the nomral vector
    n_raw = normal_vector
    magnitude_n = np.linalg.norm(n_raw)
    
    if np.isclose(magnitude_n, 0):  # Safety check
        raise ValueError("Normal vector cannot be a zero vector.")

    n_unit = n_raw / magnitude_n
    a, b, _ = n_unit

    # Build the local X-axis (u)
    if np.isclose(a, 0) and np.isclose(b, 0):  # Standard coordinate system fallback
        u_raw = np.array([1, 0, 0])
    else:
        u_raw = np.array([-b, a, 0])

    magnitude_u = np.linalg.norm(u_raw)
    u_unit = u_raw / magnitude_u

    # Build the local Y-axis (v)
    v_unit = np.cross(n_unit, u_unit)  # v = n x u

    return u_unit, v_unit


class IK_Solver:
    def __init__(self):
        pass

    def _solve(self, leg_origin:Point3D, point:Point3D):
        delta_point = point - leg_origin

        # Breadth plane
        #     Demo: https://www.desmos.com/calculator/godph2jaeb
        _, delta_y, delta_z = delta_point
        c = np.hypot(delta_y, delta_z)
        #     Safety check
        if c < _HIP_OFFSET:
            return None  # Early exit: unreachable point, too close
        
        alpha = np.arctan2(delta_z, delta_y) + _ANGLE_ZERO_OFFSETS[0]
        beta  = np.arccos((_HIP_OFFSET / c))
        
        abductor_angle = (alpha - beta) + _ANGLE_ZERO_OFFSETS[0]

        # Movement plane
        #     Demo: https://www.desmos.com/calculator/zavg5hl0gn
        movement_normal:Vector = np.array([
            0,
            _HIP_OFFSET * np.cos(abductor_angle),
            _HIP_OFFSET * np.sin(abductor_angle)
        ])

        plane_origin:Point3D = movement_normal
        u_unit, v_unit = get_unit_vectors_of_a_plane(movement_normal)

        point_relative_to_plane = delta_point - plane_origin
        local_x = np.dot(point_relative_to_plane, u_unit)
        local_z = np.dot(point_relative_to_plane, v_unit)

        r = np.hypot(local_x, local_z)  # range
        #     Safety check
        if r > _MAX_RANGE_LENGTH:
            return None  # Early exit: unreachable point, too far

        phi   = np.arctan2(local_z, local_x)
        psi   = np.arccos((np.square(_THIGH_LENGTH) + np.square(_CALF_LENGTH) - np.square(r)) / (2 * _THIGH_LENGTH * _CALF_LENGTH))  # Find angle c using law of cosines
        gamma = np.arccos((np.square(_THIGH_LENGTH) + np.square(r) - np.square(_CALF_LENGTH)) / (2 * _THIGH_LENGTH * r))             # Find angle a using law of cosines

        hip_angle   = (gamma + phi) - _ANGLE_ZERO_OFFSETS[1]
        knee_angle  = (psi - np.pi)  # Other angle on line: angle - 180

        # # TODO: Remove, for debugging
        # print(f"  delta: {np.round(delta_point, 6)}")
        # print(f"delta_y: {np.round(delta_y, 4)}")
        # print(f"delta_z: {np.round(delta_z, 4)}")
        # print(f"      d: {np.round(_HIP_OFFSET, 3)}")
        # print(f"      c: {np.round(c, 3)}")
        # print(f"  alpha: {np.round(np.degrees(alpha), 2)}°")
        # print(f"   beta: {np.round(np.degrees(beta), 2)}°")
        # print()
        # print(f"normalV: {np.round(movement_normal, 6)}")
        # print(f" p_pt_w: {np.round(point_relative_to_plane, 6)}")
        # print(f" u_unit: {np.round(u_unit, 6)}")
        # print(f" v_unit: {np.round(v_unit, 6)}")
        # print()
        # print(f"local_x: {local_x}")
        # print(f"local_z: {local_z}")
        # print(f"      r: {np.round(r, 3)}")
        # print(f"    phi: {np.round(np.degrees(phi), 2)}°")
        # print(f"    psi: {np.round(np.degrees(psi), 2)}°")
        # print(f"  gamma: {np.round(np.degrees(gamma), 2)}°")
        # print()
        # print(f"  theta_abd: {np.round(np.degrees(abductor_angle), 4)}°")
        # print(f"  theta_hip: {np.round(np.degrees(hip_angle), 4)}°")
        # print(f"  theta_kne: {np.round(np.degrees(knee_angle), 4)}°")
        # print()
        # print(f"   d_target: {np.round(delta_point, 6)}")
        # print()
        return abductor_angle, hip_angle, knee_angle

    def _clamp_motor_positions(self):
        pass
