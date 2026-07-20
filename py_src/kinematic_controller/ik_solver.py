import numpy as np
from data_structures import Point3D, Vector, Point3DList
from data_structures import AngleLimits, LegPoseList
from kinematic_controller.gait_definition import LEG_COUNT
from typing import Tuple


# Zero offsets for angles #
#     Based on: https://support.unitree.com/home/en/developer
_ANGLE_ZERO_OFFSETS = np.array([
     np.pi/2,  # Abductor 90 deg
    -np.pi/2,  # Hip     -90 deg
     0,        # Knee      0 deg
])  # In Radians

# Leg offsets from body origin #
#     Extracted from https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/go2/go2.xml
_LEG_OFFSETS_FROM_BODY_ORIGIN = np.array([
    [ 0.1934,  0.0465, 0.0],  # FL
    [ 0.1934, -0.0465, 0.0],  # FR
    [-0.1934,  0.0465, 0.0],  # BL
    [-0.1934, -0.0465, 0.0]   # BR
], dtype=float)

# Link Lengths in meters #
_HIP_OFFSET   = 0.01  # TODO: Placeholder, find real value  # CANNOT BE ZERO
_THIGH_LENGTH = 0.213
_CALF_LENGTH  = 0.213

# Rotation limits (min, max) in Radians #
_HIP_ABDUCTOR_ROT_RANGE = AngleLimits(-1.0472,  1.0472)   # approx. -60  to  60  deg
_FRONT_HIP_ROT_RANGE    = AngleLimits(-1.5708,  3.4907)   # approx. -90  to  200 deg
_BACK_HIP_ROT_RANGE     = AngleLimits(-0.5236,  4.5379)   # approx. -30  to  260 deg
_KNEE_ROT_RANGE         = AngleLimits(-2.7227, -0.83776)  # approx. -155 to -48  deg

# Output torque limits in Newton-meters #
_KNEE_TORQUE_LIMIT = (-45.43, 45.43)

# Accuracy #
#     based on input point accuracy
_INPUT_ACCURACY = 5  # d.p. of meter | e.g. 5 = 10 microns
_ANGLE_ACCURACY = 5  # d.p. of radian

# Reachability limits #
_MAX_RANGE_LENGTH = np.sqrt(np.square(_THIGH_LENGTH) + np.square(_CALF_LENGTH) - (2 * _THIGH_LENGTH * _CALF_LENGTH * np.cos(np.pi - _KNEE_ROT_RANGE[1])))
_MAX_RANGE_LENGTH = np.round(_MAX_RANGE_LENGTH, _INPUT_ACCURACY)


type LegPose = Tuple[float, float, float]  # Angles (abd, hip, knee)


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

    def _solve_internal(self, leg_origin:Point3D, point:Point3D) -> None|LegPose:
        delta_point = point - leg_origin

        # Breadth plane
        #     Demo: https://www.desmos.com/calculator/godph2jaeb
        _, delta_y, delta_z = delta_point
        c = np.hypot(delta_y, delta_z)
        #     Safety check
        if c < _HIP_OFFSET:
            print("IK failed - unreachable point! (Too close)")
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
        if np.round(r, _INPUT_ACCURACY) > _MAX_RANGE_LENGTH:
            print("IK failed - unreachable point! (Too far)")
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
        print(f"  theta_abd: {np.round(np.degrees(abductor_angle), 4)}°")
        print(f"  theta_hip: {np.round(np.degrees(hip_angle), 4)}°")
        print(f"  theta_kne: {np.round(np.degrees(knee_angle), 4)}°")
        print()
        print(f"   d_target: {np.round(delta_point, 6)}")
        print()
        return abductor_angle, hip_angle, knee_angle

    def _solve_leg(self, leg_origin:Point3D, point:Point3D, is_front_leg:bool) -> None|LegPose:
        # Solve angles
        result = self._solve_internal(leg_origin, point)
        if result is None:
            return None
        
        # Unpack angles
        abductor_angle, hip_angle, knee_angle = result

        # Ensure angles are within limits
        rounded_abductor_angle = np.round(abductor_angle, _ANGLE_ACCURACY)
        rounded_hip_angle      = np.round(hip_angle, _ANGLE_ACCURACY)
        rounded_knee_angle     = np.round(knee_angle, _ANGLE_ACCURACY)
        
        if rounded_abductor_angle < _HIP_ABDUCTOR_ROT_RANGE.minimum:
            print(f"IK fail - joint angle limit hit! (abductor.min; {rounded_abductor_angle} >= {_HIP_ABDUCTOR_ROT_RANGE.minimum})")
            return None
        if rounded_abductor_angle > _HIP_ABDUCTOR_ROT_RANGE.maximum:
            print(f"IK fail - joint angle limit hit! (abductor.min; {rounded_abductor_angle} <= {_HIP_ABDUCTOR_ROT_RANGE.maximum})")
            return None
        
        _hip_rot_range = _FRONT_HIP_ROT_RANGE if is_front_leg else _BACK_HIP_ROT_RANGE
        if rounded_hip_angle < _hip_rot_range.minimum:
            print(f"IK fail - joint angle limit hit! (hip.min; {rounded_hip_angle} >= {_hip_rot_range.minimum})")
            return None
        if rounded_hip_angle > _hip_rot_range.maximum:
            print(f"IK fail - joint angle limit hit! (hip.min; {rounded_hip_angle} <= {_hip_rot_range.maximum})")
            return None
        
        if rounded_knee_angle < _KNEE_ROT_RANGE.minimum:
            print(f"IK fail - joint angle limit hit! (knee.min; {rounded_knee_angle} >= {_KNEE_ROT_RANGE.minimum})")
            return None
        if rounded_knee_angle > _KNEE_ROT_RANGE.maximum:
            print(f"IK fail - joint angle limit hit! (knee.min; {rounded_knee_angle} <= {_KNEE_ROT_RANGE.maximum})")
            return None

        return result

    # TODO: Implement
    def solve(self, leg_points:Point3DList) -> None|LegPoseList:
        if len(leg_points) > LEG_COUNT:
            raise IndexError(f"Too many leg points! ({len(leg_points)} == {LEG_COUNT})")
        if len(leg_points) < LEG_COUNT:
            raise IndexError(f"Not enough leg points! ({len(leg_points)} == {LEG_COUNT})")

        for i in range(LEG_COUNT):
            pass

        # TODO: Check for limb collision