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

## IMU experiment logs

`run-dog` starts an IMU CSV logger automatically. By default it writes a
timestamped file under `logs/`. Choose a path and sampling interval when
needed:

```bash
run-dog --imu-log logs/trot_trial_01.csv --imu-interval 0.01
```

The CSV contains UTC timestamps, elapsed experiment time, and accelerometer
(`m/s^2`) and gyroscope (`rad/s`) axes. This works with both the Unitree
backend and the MuJoCo simulator. Samples are flushed as they are written,
and the file is closed cleanly when the controller stops.

The EKF logger is also started automatically. It records raw IMU values, foot
forces and contact flags, EKF roll/pitch/tilt and velocity estimates,
covariance values, stability status, and tipping events:

```bash
run-dog --ekf-log logs/trial_01_ekf.csv --ekf-interval 0.01 \
    --ekf-contact-threshold 20
plot-ekf logs/trial_01_ekf.csv --save plots/trial_01_ekf.png --no-show
```

To graph a completed experiment:

```bash
plot-imu logs/trot_trial_01.csv --save plots/trot_trial_01.png --no-show
```

Omit `--save` to only open the interactive Matplotlib window, or omit
`--no-show` to both save the image and display it.
