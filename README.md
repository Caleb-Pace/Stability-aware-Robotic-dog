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

The stability estimator is implemented in `py_src/EKF/stability_ekf.py`.
`UnitreeGo2StabilityEKF` estimates roll, pitch, velocity, and stability status
from IMU data and foot contacts. It does not command joints or generate a gait.

```python
from EKF.stability_ekf import UnitreeGo2StabilityEKF

estimator = UnitreeGo2StabilityEKF(dt=0.005)
estimate = estimator.step_low_state(output.get_low_state())
print(estimate.roll, estimate.pitch, estimate.contact_count, estimate.status)
```

To run the estimator alongside the robot controller, enable it explicitly:

```bash
PYTHONPATH=py_src python -m main --backend robot --interface eth0 --estimate-state
```

The controller reads the Unitree low-state IMU and foot-force contacts while
the gait is running and prints the filtered attitude, velocity, contact count,
and stability status. Verify the contact-force threshold for the installed
feet before using the stability status for recovery decisions.

The low-level recovery monitor is exposed through the installed project entry
point. It reads `rt/lowstate` through `UnitreeGo2Output` and publishes future
joint actions through the same output layer once the action builder is added:

```bash
PYTHONPATH=py_src python -m EKF.go2_highcmd_recovery_node --interface eth0
```

Run the sensor-free checks with:

```powershell
$env:PYTHONPATH = "py_src"
python -m unittest tests.test_state_estimator -v
```
