import math
import numpy as np
from data_structures import Point3D, Vector
from typing import Tuple


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


def _get_magnitude_of_a_vector(v:Vector) -> float:
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def get_unit_vectors_of_a_plane(normal_vector:Vector) -> Tuple[Vector, Vector]:
    # Normalise the nomral vector
    n_raw = normal_vector
    magnitude_n = _get_magnitude_of_a_vector(n_raw)
    
    if magnitude_n == 0:  # Safety check
        raise ValueError("Normal vector cannot be a zero vector.")

    n_unit = n_raw / magnitude_n
    a, b, _ = n_unit

    # Build the local X-axis (u)
    if np.isclose(a, 0) and np.isclose(b, 0):  # Standard coordinate system fallback
        u_raw = np.array([1, 0, 0])
    else:
        u_raw = np.array([-b, a, 0])
    u_unit = u_raw / _get_magnitude_of_a_vector(u_raw)

    # Build the local Y-axis (v)
    v_unit = np.cross(n_unit, u_unit)  # v = n x u

    return u_unit, v_unit


class IK_Solver:
    def __init__(self):
        self.maximum_move_length = (_THIGH_LENGTH) + (_CALF_LENGTH * _KNEE_ROT_RANGE[1])
        pass

    # # Adapted from: https://github.com/maanas444/go2-simulation/blob/main/mujoco/go2_lowlevel.py#L99
    # def solve(self, px, pz):
    #     r = math.sqrt(px*px + pz*pz)
    #     r = float(np.clip(r, 0.05, _THIGH_LENGTH + _CALF_LENGTH - 0.005))
    #     cos_c = (_THIGH_LENGTH**2 + _CALF_LENGTH**2 - r**2) / (2.0*_THIGH_LENGTH*_CALF_LENGTH)
    #     calf  = -(math.pi - math.acos(float(np.clip(cos_c, -1.0, 1.0))))
    #     alpha = math.atan2(px, -pz)
    #     cos_b = (_THIGH_LENGTH**2 + r**2 - _CALF_LENGTH**2) / (2.0*_THIGH_LENGTH*r)
    #     thigh = alpha + math.acos(float(np.clip(cos_b, -1.0, 1.0)))
    #     return (float(np.clip(thigh, *_FRONT_HIP_ROT_RANGE)),
    #             float(np.clip(calf,  *_KNEE_ROT_RANGE)))

    def _solve(self, leg_origin:Point3D, point:Point3D):
        delta_x, delta_y, delta_z = point - leg_origin

        abductor_angle = 0.0
        hip_angle = 0.0
        knee_angle = 0.0

        # Breadth plane
        #     Demo: https://www.desmos.com/calculator/94rhdsakta
        c = np.sqrt(np.power(delta_z, 2), np.power(delta_y, 2))
        
        alpha = np.pi/2 + np.arctan2(delta_y, delta_z)  # 90 deg + angle
        beta  = np.pi/2 - np.arctan2(_HIP_OFFSET, c)    # 90 deg - angle
        
        abductor_angle = alpha + beta

        print(f"delta_y: {np.round(delta_y, 2)}")
        print(f"delta_z: {np.round(delta_z, 2)}")
        print(f"  theta: {np.round(np.degrees(abductor_angle), 2)}")


        # Movement plane
        movement_normal:Vector = np.array([
            0,
            _HIP_OFFSET * np.cos(abductor_angle),
            _HIP_OFFSET * np.sin(abductor_angle)
        ])

        # u_unit, v_unit = 
        alpha = 0
        beta  = 0
        gamma = 0

        pass

    def _clamp_motor_positions(self):
        pass
