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

### 4. Install the MuJoCo Go2 model

The simulator uses the Go2 model from Unitree's `unitree_mujoco` repository:

```
git clone https://github.com/unitreerobotics/unitree_mujoco.git ~/unitree_mujoco
```

If the repository is stored elsewhere, point the controller at its scene file
before starting it:

```
export UNITREE_MUJOCO_SCENE=/path/to/unitree_mujoco/unitree_robots/go2/scene.xml
python3 py_src/main.py
```

While the simulator is running, `main.py` prints live EKF telemetry in the
terminal every 0.1 seconds, including status, roll, pitch, and `FL`, `FR`,
`RL`, and `RR` contact flags.

### 5. Test the stability EKF

Run the EKF terminal test independently of the controller and hardware:

```
python3 py_src/EKF/stability_ekf.py
```

It feeds the EKF level, rolling, pitching, and unrecoverable synthetic sensor
scenarios and prints the four foot contacts, estimated roll and pitch, and
stability status. Increase the filter updates if needed:

```
python3 py_src/EKF/stability_ekf.py --iterations 50
```

