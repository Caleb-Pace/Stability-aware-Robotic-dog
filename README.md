# Stability-aware-Robotic-dog
Capstone Engineering project with the Unitree Go2 (Edu). Making a stability-aware system using conventional controls, inverse kinematics, and control theory.


## Setup
### 1. Create your virtual environment (venv)
Linux & macOS:
```
python3 -m venv .venv
```

Windows:
```
py -3.12 -m venv .venv
```

The project currently supports Python 3.12 and 3.13. If `.venv` was
previously created with another Python version, recreate it with a supported
interpreter before installing the project.

### 2. Activate your virtual environment
Linux & macOS:
```
source .venv/bin/activate
```

Windows *(Command Prompt)*:
```
.venv\Scripts\activate.bat
```

Windows *(PowerShell)*:
```
.\.venv\Scripts\Activate.ps1
```

### 3. Install dependencies
```
python -m pip install -e .
```

### 4. Run the controller
After activating the virtual environment, run:
```
run-dog
```

On Windows PowerShell, the complete setup is:
```
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
run-dog
```

If PowerShell cannot find `run-dog`, the project was not installed into the
currently active virtual environment. Run `python -m pip install -e .` again
after confirming that `python --version` reports a supported version.

### Joystick movement
When the robot is standing, the left stick continuously commands motion:

- Push the left stick forward/backward to walk forward/backward.
- Push it left/right to strafe.
- Push the right stick left/right to turn.
- The farther a stick is pushed, the faster the gait cycles.
- Releasing the sticks returns the robot once to its default standing pose;
  small stick readings near center are ignored to prevent repeated transitions.

The A button still toggles between sit and stand, and the B button still runs
the turn test action. Movement is sampled continuously rather than queuing a
fixed-distance move for each joystick input. The controller starts in the
sitting pose and uses a slightly raised standing height to keep the knees more
open and improve support under load.

On real hardware, the stability monitor also latches a recovery request until
the robot has remained stable briefly, preventing repeated recovery commands
while it is settling.
