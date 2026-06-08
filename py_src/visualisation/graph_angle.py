import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from kinematic_controller.ik_solver import _HIP_OFFSET, _THIGH_LENGTH, _CALF_LENGTH
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE
from data_structures import Point3D


limits_dtype = np.dtype([('min', 'f8'), ('max', 'f8')])
JointLimitsArray = npt.NDArray[np.void]


def calculate_joint_positions(origin:Point3D, angles:npt.NDArray[np.float64]):
    # Breadth (yz) plane
    hip_abductor_joint_pos:Point3D = origin

    # Movement (xz) plane
    hip_joint_pos:Point3D = origin
    knee_joint_pos:Point3D = origin

    return hip_abductor_joint_pos, hip_joint_pos, knee_joint_pos

def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:JointLimitsArray|None = None):
    if len(angles) != 3:
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    if joint_limits and len(joint_limits) != 3:
        raise IndexError(f"3 joint limits must be provided! ({len(joint_limits)} != 3)")

    print(repr(calculate_joint_positions(origin, angles)))  # TODO: Remove for debugging

# IK Test
def main():
    origin = np.array([0, 0, 0], dtype=np.float64)
    angles = np.array([1.4, 2, 0.94], dtype=np.float64)
    joint_limits = np.array([(-1.31, 2.2), (3.14, -0.5), (0.0, 1.1)], dtype=limits_dtype)

    show_leg(origin, angles)