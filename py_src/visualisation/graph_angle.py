import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from kinematic_controller.ik_solver import _HIP_OFFSET, _THIGH_LENGTH, _CALF_LENGTH
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE
from data_structures import Point3D


limits_dtype = np.dtype([('min', 'f8'), ('max', 'f8')])
JointLimitsArray = npt.NDArray[np.void]


def _spherical_to_cartesian_coordinate(distance_r:float, azimuth_angle:float, polar_angle:float, start_point:Point3D|None = None) -> Point3D:
    if start_point is None:
        start_point = np.array([0,0,0])
    
    return start_point + np.array([
        (distance_r * np.sin(polar_angle) * np.cos(azimuth_angle)),
        (distance_r * np.sin(polar_angle) * np.sin(azimuth_angle)),
        (distance_r * np.cos(polar_angle)),
    ])

def _calculate_joint_positions(origin:Point3D, angles:npt.NDArray[np.float64]):
    abductor_theta, hip_theta, knee_theta = angles

    # Breadth (yz) plane
    abductor_pos:Point3D = origin
    theta          = angles[0]
    movement_plane = np.array([
        1,
        _HIP_OFFSET * np.sin(theta),
        _HIP_OFFSET * np.cos(theta)
    ])

    # Movement plane
    offset = np.array([
        0, 0, 0
    ])
    hip_pos:Point3D  = abductor_pos + (offset * movement_plane)
    knee_pos:Point3D = hip_pos

    #    End effector pos
    foot_pos:Point3D = knee_pos

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
    angles = np.array([1.4, 2, 0.94], dtype=np.float64)
    joint_limits = np.array([(-1.31, 2.2), (3.14, -0.5), (0.0, 1.1)], dtype=limits_dtype)

    show_leg(origin, angles)