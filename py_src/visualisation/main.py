#!/usr/bin/env python3
import interpolation.lagrange
from visualisation.graph_3d_trajectory import show_trajectory

def main():
    show_trajectory(interpolation.lagrange.Lagrange())

if __name__ == "__main__":
    main()
