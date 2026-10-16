import numpy as np
from data_structures.leg_trajectories import LegTrajectories
from data_structures.constants import SIT_PITCH, SIT_HEIGHT, STAND_HEIGHT


SIT = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([STAND_HEIGHT, SIT_HEIGHT], dtype=float),
    orientations       = np.array([[0, 0, 0], [0, SIT_PITCH, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)

STAND = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([SIT_HEIGHT, STAND_HEIGHT], dtype=float),
    orientations       = np.array([[0, SIT_PITCH, 0], [0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)

HALF_STAND = LegTrajectories(
    sample_count       = 24,
    heights            = np.array([(STAND_HEIGHT-0.15), STAND_HEIGHT], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)


BACK_TO_SIDE = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([SIT_HEIGHT, 0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0], [ -0.1,   -0.01,  0.09]]),
        np.array([[0.0, 0.0, 0.0], [ -0.205,  0.01,  0.05]]),
        np.array([[0.0, 0.0, 0.0], [ -0.1,   -0.01,  0.09]]),
        np.array([[0.0, 0.0, 0.0], [ -0.205,  0.01,  0.05]]),
    ], dtype=object),
    allow_below_ground_plane=True
)
N0 = LegTrajectories(
    sample_count       = 8,
    heights            = np.array([0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[ -0.1,   -0.01,  0.09]]),
        np.array([[ -0.205,  0.01,  0.05]]),
        np.array([[ -0.1,   -0.01,  0.09]]),
        np.array([[ -0.205,  0.01,  0.05]]),
    ], dtype=object),
    allow_below_ground_plane=True
)
N1 = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[ -0.1,   -0.01,  0.09], [ -0.1,     0.01, -0.3]]),
        np.array([[ -0.205,  0.01,  0.05], [ -0.205,  -0.01,  0.05]]),
        np.array([[ -0.1,   -0.01,  0.09], [ -0.1,     0.01, -0.3]]),
        np.array([[ -0.205,  0.01,  0.05], [ -0.205,  -0.01,  0.05]]),
    ], dtype=object),
    allow_below_ground_plane=True
)
N2 = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[ -0.1,     0.01, -0.3], [ -0.1,     0.015, -0.25]]),
        np.array([[ -0.205,  -0.01,  0.05], [ -0.205, -0.015,  0.05]]),
        np.array([[ -0.1,     0.01, -0.3], [ -0.1,     0.015, -0.25]]),
        np.array([[ -0.205,  -0.01,  0.05], [ -0.205, -0.015,  0.05]]),
    ], dtype=object),
    allow_below_ground_plane=True
)
N3 = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[-0.1,     0.015, -0.25], [-0.1,   0.015, -0.08]]),
        np.array([[-0.205,  -0.015,  0.05], [-0.05,  -0.05,  0.05]]),
        np.array([[-0.1,     0.015, -0.25], [-0.1,   0.015, -0.08]]),
        np.array([[-0.205,  -0.015,  0.05], [-0.05,  -0.05,  0.05]]),
    ], dtype=object),
    allow_below_ground_plane=True
)
N4 = LegTrajectories(
    sample_count       = 48,
    heights            = np.array([0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[-0.1,   0.015, -0.08], [-0.1,  0.0, -0.05], [0.0, 0.0, 0.0]]),
        np.array([[-0.05,  -0.05,  0.05], [0.0, 0.0,  -0.02], [0.0, 0.0, 0.0]]),
        np.array([[-0.1,   0.015, -0.08], [-0.1,  0.0, -0.05], [0.0, 0.0, 0.0]]),
        np.array([[-0.05,  -0.05,  0.05], [0.0, 0.0,  -0.02], [0.0, 0.0, 0.0]]),
        # np.array([[-0.1,   0.015, -0.2], [-0.1, 0.0, -0.20]]),
        # np.array([[-0.05, -0.05,  0.05], [ -0.15, -0.03,  0.05]]),
        # np.array([[-0.1,   0.015, -0.2], [-0.1, 0.0, -0.20]]),
        # np.array([[-0.05, -0.05,  0.05], [ -0.15, -0.03,  0.05]]),
        # np.array([[-0.1,   0.015, -0.25], [-0.1, 0.015, -0.20]]),
        # np.array([[-0.05, -0.05,  0.05], [0.0,  -0.05,  0.0]]),
        # np.array([[-0.1,   0.015, -0.25], [-0.1, 0.015, -0.20]]),
        # np.array([[-0.05, -0.05,  0.05], [0.0,  -0.05,  0.0]]),
    ], dtype=object),
    allow_below_ground_plane=True
)

GET_UP_FROM_FALL = LegTrajectories(
    sample_count       = 48,
    heights            = np.array([SIT_HEIGHT, 0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        # np.array([[0.0, 0.0, 0.0], [-0.175,  -0.01, 0.09]]),
        # np.array([[0.0, 0.0, 0.0], [-0.205,  -0.01, 0.05]]),
        
        np.array([[0.0, 0.0, 0.0], [ -0.1,   -0.01,  0.09], [ -0.1,     0.01, -0.3]]),
        np.array([[0.0, 0.0, 0.0], [ -0.205,  0.01,  0.05], [ -0.205,  -0.01,  0.05]]),
        np.array([[0.0, 0.0, 0.0], [ -0.1,   -0.01,  0.09], [ -0.1,     0.01, -0.3]]),
        np.array([[0.0, 0.0, 0.0], [ -0.205,  0.01,  0.05], [ -0.205,  -0.01,  0.05]]),
        
        # np.array([[0.0, 0.0, 0.0], [ 0, 0, -0.1]]),
        # np.array([[0.0, 0.0, 0.0], [ -0.05,   0,  -0.1]]),
        # np.array([[0.0, 0.0, 0.0], [ 0, 0, -0.1]]),
        # np.array([[0.0, 0.0, 0.0], [ -0.05,   0,  -0.1]]),

        # np.array([[0.0, 0.0, 0.0], [ 0.0,   -0.02,  0.09]]),
        # np.array([[0.0, 0.0, 0.0], [-0.174,  0.00, -0.09]]),
    ], dtype=object),
    allow_below_ground_plane=True
)

CATCH_BASIC = LegTrajectories(
    sample_count       = 24,
    heights            = np.array([(SIT_HEIGHT+0.1), STAND_HEIGHT], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)


TURN_RIGHT = LegTrajectories(
    sample_count       = 64,
    heights            = np.array([STAND_HEIGHT], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0], [ 0.025, 0.0, 0.08], [ 0.05, 0.0, 0.1], [ 0.075, 0.0, 0.08], [ 0.1, 0.0, 0.0], [ 0.075, 0.0, 0.0], [ 0.05, 0.0, 0.0], [ 0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0], [-0.025, 0.0, 0.08], [-0.05, 0.0, 0.1], [-0.075, 0.0, 0.08], [-0.1, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.05, 0.0, 0.0], [-0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0], [ 0.025, 0.0, 0.08], [ 0.05, 0.0, 0.1], [ 0.075, 0.0, 0.08], [ 0.1, 0.0, 0.0], [ 0.075, 0.0, 0.0], [ 0.05, 0.0, 0.0], [ 0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0], [-0.025, 0.0, 0.08], [-0.05, 0.0, 0.1], [-0.075, 0.0, 0.08], [-0.1, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.05, 0.0, 0.0], [-0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    ], dtype=object)
)

TWISTING_TEST = LegTrajectories(
    sample_count       = 256,
    heights            = np.array([STAND_HEIGHT], dtype=float),
    orientations       = np.array([
        [0, 0, 0], [-np.pi/2, 0, 0],
        [0, 0, 0], [np.pi/2, 0, 0],
        [0, 0, 0], [0, -np.pi/4, 0],
        [0, 0, 0], [0, 0, -np.pi/6],
        [0, 0, 0], [0, 0, np.pi/6],
        [0, 0, 0]
    ], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)

YEET = LegTrajectories(
    sample_count       = 2,
    heights            = np.array([SIT_HEIGHT, 0.386], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)
