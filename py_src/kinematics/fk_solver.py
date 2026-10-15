import numpy as np
import numpy.typing as npt

from typing import Tuple
from data_structures import Point2D, Point3D, Vector
from data_structures import AngleLimits, TorqueLimits

# Zero offsets for angles #
#     Based on: https://support.unitree.com/home/en/developer
_ANGLE_ZERO_OFFSETS = np.array([
     np.pi/2,  # Abductor 90 deg
    -np.pi/2,  # Hip     -90 deg
     0,        # Knee      0 deg
])  # In Radians

# Leg offsets from body origin #
#     Extracted from https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml
LEG_OFFSETS_FROM_BODY_ORIGIN = np.array([
    [ 0.1934, -0.0465, 0.0],  # FR
    [ 0.1934,  0.0465, 0.0],  # FL
    [-0.1934, -0.0465, 0.0],  # RR
    [-0.1934,  0.0465, 0.0]   # RL
], dtype=float)

# Link Lengths in meters #
#     Extracted from https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml
_HIP_OFFSET   = 0.01  # TODO: Placeholder, find real value  # CANNOT BE ZERO
_THIGH_LENGTH = 0.213
_CALF_LENGTH  = 0.213

# Rotation limits (min, max) in Radians #
#     Extracted from https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml
_HIP_ABDUCTOR_ROT_RANGE = AngleLimits(-1.0472,  1.0472)   # approx. -60  to  60  deg
_FRONT_HIP_ROT_RANGE    = AngleLimits(-1.5708,  3.4907)   # approx. -90  to  200 deg
_BACK_HIP_ROT_RANGE     = AngleLimits(-0.5236,  4.5379)   # approx. -30  to  260 deg
_KNEE_ROT_RANGE         = AngleLimits(-2.7227, -0.83776)  # approx. -155 to -48  deg

# Output torque limits in Newton-meters #
#     Extracted from https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml
_HIP_ABDUCTOR_TORQUE_LIMIT = TorqueLimits(-23.7, 23.7)  # Nm
_HIP_TORQUE_LIMIT          = _HIP_ABDUCTOR_TORQUE_LIMIT  # Nm
_KNEE_TORQUE_LIMIT         = TorqueLimits(-45.43, 45.43)  # Nm

# Accuracy #
#     based on input point accuracy
_INPUT_ACCURACY = 5  # d.p. of meter | e.g. 5 = 10 microns
_ANGLE_ACCURACY = 5  # d.p. of radian

# Reachability limits #
_MAX_RANGE_LENGTH = np.sqrt(np.square(_THIGH_LENGTH) + np.square(_CALF_LENGTH) - (2 * _THIGH_LENGTH * _CALF_LENGTH * np.cos(np.pi - _KNEE_ROT_RANGE[1])))
_MAX_RANGE_LENGTH = np.round(_MAX_RANGE_LENGTH, _INPUT_ACCURACY)


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


def degrees_to_radians(deg:float) -> float:
    return deg * (np.pi / 180)

def _polar_to_cartesian_coordinate(distance:float, angle:float) -> Point2D:
    return np.array([
        distance * np.cos(angle),
        distance * np.sin(angle)
    ])

def _spherical_to_cartesian_coordinate(distance_r:float, azimuth_angle:float, polar_angle:float, start_point:Point3D|None = None) -> Point3D:
    if start_point is None:
        start_point = np.array([0,0,0])

    return start_point + np.array([
        (distance_r * np.sin(polar_angle) * np.cos(azimuth_angle)),
        (distance_r * np.sin(polar_angle) * np.sin(azimuth_angle)),
        (distance_r * np.cos(polar_angle)),
    ])

def _convert_local_to_world_coordinate(local_coordinate:Point2D, plane_anchor_point:Point3D, u_unit:Vector, v_unit:Vector) -> Point3D:
    x_prime, y_prime = local_coordinate
    return plane_anchor_point + (x_prime * u_unit) + (y_prime * v_unit)

def calculate_joint_positions(origin:Point3D, angles:npt.NDArray[np.float64], is_left_side:bool = False):
    AZIMUTH_POS_Y_ANGLE = np.pi/2  # 90 deg
    
    if len(angles) != 3:  # Safety check
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    angles += _ANGLE_ZERO_OFFSETS  # Apply angle offsets
    abductor_angle, hip_angle, knee_relative_angle = angles

    # Apply left side offsets
    if is_left_side:
        AZIMUTH_POS_Y_ANGLE += np.pi  # + 180 deg

        # Invert angles
        hip_angle           = np.pi - hip_angle
        knee_relative_angle = -knee_relative_angle

    # Breadth (yz) plane
    abductor_pos:Point3D = origin

    # Movement plane
    movement_plane_anchor_point:Point3D = _spherical_to_cartesian_coordinate(_HIP_OFFSET, AZIMUTH_POS_Y_ANGLE, abductor_angle, abductor_pos)
    movement_plane_normal_vector:Vector = movement_plane_anchor_point
    u_unit, v_unit                      = get_unit_vectors_of_a_plane(movement_plane_normal_vector)

    #    Hip position
    hip_pos:Point3D = movement_plane_anchor_point

    #    Knee position
    knee_local_pos:Point2D = _polar_to_cartesian_coordinate(_THIGH_LENGTH, hip_angle)
    knee_pos:Point3D       = _convert_local_to_world_coordinate(knee_local_pos, hip_pos, u_unit, v_unit)

    #    Foot (End effector) position
    knee_absolute_angle = hip_angle + knee_relative_angle
    foot_local_pos:Point2D = _polar_to_cartesian_coordinate(_CALF_LENGTH, knee_absolute_angle)
    foot_pos:Point3D       = _convert_local_to_world_coordinate(foot_local_pos, knee_pos, u_unit, v_unit)

    return abductor_pos, hip_pos, knee_pos, foot_pos
