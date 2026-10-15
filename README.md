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
python -m venv .venv
```

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
pip install .
```
***Note:** use the editable flag (`-e`) for development.*

## Dynamics safety

The controller applies a conservative Go2 mass-property support model during
stance. It distributes the mapped robot weight across feet whose planned
height is at ground level, converts the vertical support force to joint torque
with the leg Jacobian, and applies no dynamics feed-forward to swing legs.
Feed-forward torque is limited to 35% of the published joint torque limits and
slew-limited before it reaches either the simulator or the robot output layer.
The simulator also clamps the combined PID and feed-forward torque to the
published limits.
