import math
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from data_structures import Point3D
from kinematic_controller.ik_solver import _HIP_OFFSET, _THIGH_LENGTH, _CALF_LENGTH
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE
from typing import Annotated, Tuple

limits_dtype = np.dtype([('min', 'f8'), ('max', 'f8')])
JointLimitsArray = npt.NDArray[np.void]


type Point2D = Annotated[npt.NDArray[np.float64], (2,)]
type Plane   = Annotated[npt.NDArray[np.float64], (4,)]
type Vector  = Point3D


def _degrees_to_radians(deg:float) -> float:
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

def _get_magnitude_of_a_vector(v:Vector) -> float:
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def _get_unit_vectors_of_a_plane(normal_vector:Vector) -> Tuple[Vector, Vector]:
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

def _convert_local_to_world_coordinate(local_coordinate:Point2D, plane_anchor_point:Point3D, u_unit:Vector, v_unit:Vector) -> Point3D:
    if len(local_coordinate) != 2:  # Safety check
        raise ValueError(f"Invalid plane coordinate! (axes count: {len(local_coordinate)} != {2})")
    
    x_prime, y_prime = local_coordinate
    return plane_anchor_point + (x_prime * u_unit) + (y_prime * v_unit)

def _calculate_joint_positions(origin:Point3D, angles:npt.NDArray[np.float64]):
    POLAR_POS_Y_ANGLE = _degrees_to_radians(90)
    
    abductor_angle, hip_angle, knee_relative_angle = angles

    # Breadth (yz) plane
    abductor_pos:Point3D = origin

    # Movement plane
    movement_plane_anchor_point:Point3D = _spherical_to_cartesian_coordinate(_HIP_OFFSET, abductor_angle, POLAR_POS_Y_ANGLE, abductor_pos)
    movement_plane_normal_vector:Vector = movement_plane_anchor_point
    u_unit, v_unit                      = _get_unit_vectors_of_a_plane(movement_plane_normal_vector)

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

# TODO: Add show movement plane option
def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:JointLimitsArray|None = None):
    if len(angles) != 3:
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    if joint_limits and len(joint_limits) != 3:
        raise IndexError(f"3 joint limits must be provided! ({len(joint_limits)} != 3)")

    print(repr(_calculate_joint_positions(origin, angles)))  # TODO: Remove for debugging

# IK Test
def main():
    origin = np.array([0, 0, 0], dtype=np.float64)
    angles = np.array([
        _degrees_to_radians(90),
        _degrees_to_radians(55),
        _degrees_to_radians(60)
    ], dtype=np.float64)
    joint_limits = np.array([(-1.31, 2.2), (3.14, -0.5), (0.0, 1.1)], dtype=limits_dtype)

    show_leg(origin, angles)