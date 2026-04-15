#!/usr/bin/env python3
from kinematic_controller.stepper import step
from kinematic_controller.gaits import TROT

def main():
    # for i in range(0, 4):
    for i in range(0, 1):
        foot_positions = step(TROT, i)
        print(f"{repr(foot_positions)}")

if __name__ == "__main__":
    main()
