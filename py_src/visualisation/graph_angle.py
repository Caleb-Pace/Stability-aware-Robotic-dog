import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from data_structures import Point3D
from kinematic_controller.fk_solver import degrees_to_radians, calculate_joint_positions
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE

limits_dtype = np.dtype([('min', 'f8'), ('max', 'f8')])
JointLimitsArray = npt.NDArray[np.void]


# TODO: Add show movement plane option
def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:JointLimitsArray|None = None):
    if len(angles) != 3:  # Parameter check
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    if joint_limits and len(joint_limits) != 3:   # Parameter check
        raise IndexError(f"3 joint limits must be provided! ({len(joint_limits)} != 3)")

    print(repr(calculate_joint_positions(origin, angles)))  # TODO: Remove for debugging

# IK Test
def main():
    origin = np.array([0, 0, 0], dtype=np.float64)
    angles = np.array([
        degrees_to_radians(90),
        degrees_to_radians(55),
        degrees_to_radians(60)
    ], dtype=np.float64)
    joint_limits = np.array([(-1.31, 2.2), (3.14, -0.5), (0.0, 1.1)], dtype=limits_dtype)

    show_leg(origin, angles)