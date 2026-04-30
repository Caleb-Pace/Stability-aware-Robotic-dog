#!/usr/bin/env python3
# from interpolation import Lagrange
import interpolation
from visualisation.graph_3d_trajectory import show_trajectory, compare_interpolators

def main():
    # show_trajectory(interpolation.Lagrange())
    # show_trajectory(interpolation.Lagrange(), interpolation.Lagrange())
    # show_trajectory(interpolation.CatmullRomSpline(alpha=0))
    # show_trajectory(interpolation.CatmullRomSpline(alpha=0.5))
    # show_trajectory(interpolation.CatmullRomSpline(alpha=0.5), interpolation.CatmullRomSpline(alpha=0))
    # show_trajectory(interpolation.CatmullRomSpline(alpha=0.5), interpolation.CatmullRomSpline(alpha=1))
    # show_trajectory(interpolation.CatmullRomSpline(alpha=0.5), interpolation.Lagrange())
    show_trajectory(interpolation.CatmullRomSpline(), interpolation.CatmullRomSpline())
    # compare_interpolators()

if __name__ == "__main__":
    main()
