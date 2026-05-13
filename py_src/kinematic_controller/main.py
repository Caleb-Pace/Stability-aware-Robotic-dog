#!/usr/bin/env python3
from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.interpolator import get_foot_trajectories
from kinematic_controller.stepper import step

def main():
    gait:Gait = TROT

    detail = 32  # (Point/Node count)
    foot_trajectories = get_foot_trajectories(gait, detail)

    for i in range(0, detail):
        foot_positions = step(foot_trajectories, i)
        print(f"{repr(foot_positions)}")

if __name__ == "__main__":
    main()
