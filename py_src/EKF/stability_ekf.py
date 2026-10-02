from dataclasses import dataclass
from enum import Enum, auto
import numpy as np


GRAVITY = 9.81


class StabilityState(Enum):
    STABLE = auto()
    TIPPING_ROLL = auto()
    TIPPING_PITCH = auto()
    FREE_FALL = auto()
    UNRECOVERABLE = auto()


@dataclass(frozen=True)
class StabilitySensorSample:
    """Synchronized IMU and foot contact sample from Unitree Go2 LowState."""

    accelerometer: np.ndarray  # [ax, ay, az] in body frame (m/s^2)
    gyroscope: np.ndarray      # [wx, wy, wz] in body frame (rad/s)
    foot_contacts: np.ndarray  # [FL, FR, RL, RR] boolean

    def __post_init__(self):
        acc = np.asarray(self.accelerometer, dtype=np.float64)
        gyro = np.asarray(self.gyroscope, dtype=np.float64)
        contacts = np.asarray(self.foot_contacts, dtype=bool)

        if acc.shape != (3,) or gyro.shape != (3,):
            raise ValueError("Accelerometer and Gyroscope must be 3D vectors.")
        if contacts.shape != (4,):
            raise ValueError("foot_contacts must have shape (4,).")

        object.__setattr__(self, "accelerometer", acc)
        object.__setattr__(self, "gyroscope", gyro)
        object.__setattr__(self, "foot_contacts", contacts)

    @classmethod
    def from_low_state(cls, low_state, contact_force_threshold: float = 20.0):
        """Extracts IMU and contact states from Unitree Go2 SDK LowState."""
        imu = low_state.imu_state
        acc = np.array([imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]], dtype=np.float64)
        gyro = np.array([imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]], dtype=np.float64)

        raw_forces = getattr(low_state, "foot_force", np.zeros(4))
        contacts = np.asarray(raw_forces) > contact_force_threshold
        return cls(acc, gyro, contacts)


@dataclass(frozen=True)
class StabilityEstimate:
    roll: float               # Radians
    pitch: float              # Radians
    velocity: np.ndarray      # 3D world velocity [vx, vy, vz] (m/s)
    angular_velocity: np.ndarray  # Body angular rates [wx, wy, wz] (rad/s)
    tilt_angle: float         # Magnitude of tilt from vertical (radians)
    contact_count: int
    status: StabilityState
    is_tipping: bool

    @property
    def roll_deg(self) -> float:
        return float(np.degrees(self.roll))

    @property
    def pitch_deg(self) -> float:
        return float(np.degrees(self.pitch))

    @property
    def tilt_deg(self) -> float:
        return float(np.degrees(self.tilt_angle))


class UnitreeGo2StabilityEKF:
    """EKF State Estimator for quadruped stability & tipping detection.

    State Vector x (6D):
        x = [roll, pitch, vx, vy, vz, z_height]
    """

    def __init__(
        self,
        dt: float = 0.002,  # Go2 LowState runs at ~500 Hz
        gravity: float = GRAVITY,
        nominal_height: float = 0.28,
    ):
        self.dt = float(dt)
        self.gravity = float(gravity)
        self.nominal_height = float(nominal_height)

        # State: [roll, pitch, vx, vy, vz, z_height]
        self.x = np.zeros(6, dtype=np.float64)
        self.x[5] = self.nominal_height

        # Covariance matrix P
        self.P = np.diag([0.05, 0.05, 0.1, 0.1, 0.1, 0.05]) ** 2

        # Process noise Q
        self.Q = np.diag([0.01, 0.01, 0.2, 0.2, 0.3, 0.05]) ** 2

        # Measurement Noise Covariances
        self.R_accel = np.eye(3) * (0.3**2)
        self.R_zero_vel = np.eye(3) * (0.05**2)
        self.R_slip_vel = np.diag([0.2, 0.2, 0.02]) ** 2
        self.R_height = 0.03**2

        # Safety / Tipping Thresholds
        self.max_stable_roll = np.radians(15.0)
        self.max_stable_pitch = np.radians(15.0)
        self.max_angular_vel = np.radians(120.0)
        self.unrecoverable_tilt = np.radians(45.0)

    @staticmethod
    def _rotation_world_from_body(roll: float, pitch: float) -> np.ndarray:
        """Rotation matrix R_W_B mapping vectors from Body frame to World frame (Yaw=0)."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)

        return np.array([
            [cp, sr * sp, cr * sp],
            [0.0, cr, -sr],
            [-sp, sr * cp, cr * cp],
        ])

    def _euler_rate_matrix(self, roll: float, pitch: float) -> np.ndarray:
        """Maps 3D body angular velocities [wx, wy, wz] to Euler rates [roll_dot, pitch_dot]."""
        cr, sr = np.cos(roll), np.sin(roll)
        tp = np.clip(np.tan(pitch), -10.0, 10.0)

        return np.array([
            [1.0, sr * tp, cr * tp],
            [0.0, cr, -sr],
        ])

    def predict(self, sample: StabilitySensorSample) -> None:
        """Predict step using gyro integration and body-to-world accelerometer transform."""
        roll, pitch = self.x[0], self.x[1]

        # 1. Non-linear attitude kinematics
        E = self._euler_rate_matrix(roll, pitch)
        euler_rates = E @ sample.gyroscope
        next_roll = roll + euler_rates[0] * self.dt
        next_pitch = pitch + euler_rates[1] * self.dt

        # 2. World acceleration
        R_W_B = self._rotation_world_from_body(next_roll, next_pitch)
        acc_world = R_W_B @ sample.accelerometer
        acc_world[2] -= self.gravity

        # 3. Position and velocity step
        next_vel = self.x[2:5] + acc_world * self.dt
        next_height = self.x[5] + next_vel[2] * self.dt

        predicted_x = np.array([next_roll, next_pitch, next_vel[0], next_vel[1], next_vel[2], next_height])

        # 4. Numerical Jacobian computation
        epsilon = 1e-6
        F = np.eye(6)
        for i in range(6):
            x_perturbed = self.x.copy()
            x_perturbed[i] += epsilon

            E_p = self._euler_rate_matrix(x_perturbed[0], x_perturbed[1])
            e_rates_p = E_p @ sample.gyroscope
            r_p = x_perturbed[0] + e_rates_p[0] * self.dt
            p_p = x_perturbed[1] + e_rates_p[1] * self.dt

            R_p = self._rotation_world_from_body(r_p, p_p)
            acc_w_p = R_p @ sample.accelerometer
            acc_w_p[2] -= self.gravity

            v_p = x_perturbed[2:5] + acc_w_p * self.dt
            h_p = x_perturbed[5] + v_p[2] * self.dt

            pred_p = np.array([r_p, p_p, v_p[0], v_p[1], v_p[2], h_p])
            F[:, i] = (pred_p - predicted_x) / epsilon

        self.x = predicted_x
        self.P = F @ self.P @ F.T + self.Q * self.dt

    def _kalman_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        y = z - h
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

    def update_gravity_and_contacts(self, sample: StabilitySensorSample) -> None:
        acc_norm = np.linalg.norm(sample.accelerometer)
        gyro_norm = np.linalg.norm(sample.gyroscope)
        n_contacts = int(np.count_nonzero(sample.foot_contacts))

        # Gravity direction update
        if abs(acc_norm - self.gravity) < 2.0 and gyro_norm < 1.5:
            R_W_B = self._rotation_world_from_body(self.x[0], self.x[1])
            expected_acc = R_W_B.T @ np.array([0.0, 0.0, self.gravity])

            eps = 1e-6
            H_acc = np.zeros((3, 6))
            for i in (0, 1):
                x_p = self.x.copy()
                x_p[i] += eps
                R_p = self._rotation_world_from_body(x_p[0], x_p[1])
                exp_p = R_p.T @ np.array([0.0, 0.0, self.gravity])
                H_acc[:, i] = (exp_p - expected_acc) / eps

            self._kalman_update(sample.accelerometer, expected_acc, H_acc, self.R_accel)

        # Contact adaptive velocity updates
        H_vel = np.zeros((3, 6))
        H_vel[0:3, 2:5] = np.eye(3)

        if n_contacts >= 3:
            self._kalman_update(np.zeros(3), self.x[2:5], H_vel, self.R_zero_vel)
            H_h = np.zeros((1, 6))
            H_h[0, 5] = 1.0
            self._kalman_update(np.array([self.nominal_height]), np.array([self.x[5]]), H_h, np.array([[self.R_height]]))
        elif n_contacts in (1, 2):
            self._kalman_update(np.zeros(3), self.x[2:5], H_vel, self.R_slip_vel)

    def classify_stability(self, sample: StabilitySensorSample) -> StabilityState:
        roll, pitch = abs(self.x[0]), abs(self.x[1])
        tilt = float(np.hypot(roll, pitch))
        acc_norm = np.linalg.norm(sample.accelerometer)
        n_contacts = np.count_nonzero(sample.foot_contacts)

        if tilt > self.unrecoverable_tilt:
            return StabilityState.UNRECOVERABLE

        if acc_norm < 2.5 and n_contacts == 0:
            return StabilityState.FREE_FALL

        roll_rate, pitch_rate = sample.gyroscope[0], sample.gyroscope[1]

        predicted_roll = abs(self.x[0] + roll_rate * 0.15)
        if predicted_roll > self.max_stable_roll or (roll > self.max_stable_roll * 0.8 and abs(roll_rate) > self.max_angular_vel):
            return StabilityState.TIPPING_ROLL

        predicted_pitch = abs(self.x[1] + pitch_rate * 0.15)
        if predicted_pitch > self.max_stable_pitch or (pitch > self.max_stable_pitch * 0.8 and abs(pitch_rate) > self.max_angular_vel):
            return StabilityState.TIPPING_PITCH

        return StabilityState.STABLE

    def step(self, sample: StabilitySensorSample) -> StabilityEstimate:
        self.predict(sample)
        self.update_gravity_and_contacts(sample)

        status = self.classify_stability(sample)
        tilt = float(np.hypot(self.x[0], self.x[1]))

        return StabilityEstimate(
            roll=float(self.x[0]),
            pitch=float(self.x[1]),
            velocity=self.x[2:5].copy(),
            angular_velocity=sample.gyroscope.copy(),
            tilt_angle=tilt,
            contact_count=int(np.count_nonzero(sample.foot_contacts)),
            status=status,
            is_tipping=status != StabilityState.STABLE,
        )

    def step_low_state(self, low_state, contact_force_threshold: float = 20.0) -> StabilityEstimate:
        return self.step(StabilitySensorSample.from_low_state(low_state, contact_force_threshold))