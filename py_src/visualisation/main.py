#!/usr/bin/env python3
import interpolation
from kinematic_controller.gaits import TROT
from visualisation.graph_3d_interpolated_curves import show_interpolated_curves, compare_interpolators
from visualisation.graph_3d_kinematics import show_full_gait, show_motor_positions, show_ik_constraints

def main():
    # show_interpolated_curves(interpolation.CatmullRomSpline(), interpolation.CatmullRomSpline())
    # compare_interpolators()

    show_full_gait(TROT)

if __name__ == "__main__":
    main()
