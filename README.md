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

## Stability estimator experiment

The first stability-awareness slice is isolated in `py_src/state_estimator.py`.
`StabilityEKF` estimates roll, pitch, and body velocity from IMU data. When one
or more feet are in contact, it applies a zero-velocity update to reduce IMU
drift. It does not command joints or generate a gait.

```python
from state_estimator import StabilityEKF

estimator = StabilityEKF(dt=0.005)
estimate = estimator.step_low_state(output.get_low_state())
print(estimate.roll, estimate.pitch, estimate.contact_count, estimate.stable)
```

Run the sensor-free checks with:

```powershell
$env:PYTHONPATH = "py_src"
python -m unittest tests.test_state_estimator -v
```
