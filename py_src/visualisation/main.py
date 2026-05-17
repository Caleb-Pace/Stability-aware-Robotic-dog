#!/usr/bin/env python3
import interpolation
from visualisation.graph_3d_interpolated_curves import show_interpolated_curves, compare_interpolators

def main():
    # show_interpolated_curves(interpolation.Lagrange())
    # show_interpolated_curves(interpolation.Lagrange(), interpolation.Lagrange())
    # show_interpolated_curves(interpolation.CatmullRomSpline(alpha=0))
    # show_interpolated_curves(interpolation.CatmullRomSpline(alpha=0.5))
    # show_interpolated_curves(interpolation.CatmullRomSpline(alpha=0.5), interpolation.CatmullRomSpline(alpha=0))
    # show_interpolated_curves(interpolation.CatmullRomSpline(alpha=0.5), interpolation.CatmullRomSpline(alpha=1))
    # show_interpolated_curves(interpolation.CatmullRomSpline(alpha=0.5), interpolation.Lagrange())
    show_interpolated_curves(interpolation.CatmullRomSpline(), interpolation.CatmullRomSpline())
    # compare_interpolators()

if __name__ == "__main__":
    main()
