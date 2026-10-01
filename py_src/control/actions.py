import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories


SIT = LegTrajectories(
    sample_count       = 32,
    heights            = np.array([0.3, 0.09], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
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
    heights            = np.array([0.09, 0.3], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)

TWISTING_TEST = LegTrajectories(
    sample_count       = 256,
    heights            = np.array([0.3], dtype=float),
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
    heights            = np.array([0.09, 0.386], dtype=float),
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
    heights            = np.array([0.3], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0], [ 0.025, 0.0, 0.08], [ 0.05, 0.0, 0.1], [ 0.075, 0.0, 0.08], [ 0.1, 0.0, 0.0], [ 0.075, 0.0, 0.0], [ 0.05, 0.0, 0.0], [ 0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0], [-0.025, 0.0, 0.08], [-0.05, 0.0, 0.1], [-0.075, 0.0, 0.08], [-0.1, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.05, 0.0, 0.0], [-0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0], [ 0.025, 0.0, 0.08], [ 0.05, 0.0, 0.1], [ 0.075, 0.0, 0.08], [ 0.1, 0.0, 0.0], [ 0.075, 0.0, 0.0], [ 0.05, 0.0, 0.0], [ 0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0], [-0.025, 0.0, 0.08], [-0.05, 0.0, 0.1], [-0.075, 0.0, 0.08], [-0.1, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.05, 0.0, 0.0], [-0.025, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    ], dtype=object)
)