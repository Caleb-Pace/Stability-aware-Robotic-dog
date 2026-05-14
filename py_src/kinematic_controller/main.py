#!/usr/bin/env python3
import math
from kinematic_controller.gaits import TROT
from kinematic_controller.gait_definition import Gait
from kinematic_controller.stepper import step

def main():
    gait:Gait = TROT

    detail = math.ceil(gait.time_reference)  # (Point/Node count)

    for i in range(0, detail):
        foot_positions = step(gait, i)
        print(f"{repr(foot_positions)}")

if __name__ == "__main__":
    main()
