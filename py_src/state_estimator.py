from dataclasses import dataclass

import numpy as np


GRAVITY = 9.81


@dataclass(frozen=True)
class StabilitySensorSample:
    """One synchronized IMU/contact sample used by the stability estimator."""

    accelerometer: np.ndarray
    gyroscope: np.ndarray
    foot_contacts: np.ndarray

    def __post_init__(self):
        accelerometer = np.asarray(self.accelerometer, dtype=np.float64)
        gyroscope = np.asarray(self.gyroscope, dtype=np.float64)
        foot_contacts = np.asarray(self.foot_contacts, dtype=bool)

        if accelerometer.shape != (3,) or gyroscope.shape != (3,):
            raise ValueError("IMU measurements must each have shape (3,)")
        if foot_contacts.shape != (4,):
            raise ValueError("foot_contacts must have shape (4,)")

        object.__setattr__(self, "accelerometer", accelerometer)
        object.__setattr__(self, "gyroscope", gyroscope)
        object.__setattr__(self, "foot_contacts", foot_contacts)

    @classmethod
    def from_low_state(cls, low_state, contact_force_threshold: float = 20.0):
        """Adapt a Unitree-like LowState without requiring the SDK at import time."""
        imu = low_state.imu_state
        accelerometer = getattr(imu, "accelerometer")
        gyroscope = getattr(imu, "gyroscope", getattr(imu, "gyro", np.zeros(3)))

        raw_contacts = None
        for name in ("foot_contact", "foot_contacts", "foot_force", "foot_force_est"):
            raw_contacts = getattr(low_state, name, None)
            if raw_contacts is not None:
                break
        if raw_contacts is None:
            raw_contacts = np.zeros(4, dtype=bool)
        raw_contacts = np.asarray(raw_contacts)
        contacts = raw_contacts.astype(bool) if raw_contacts.dtype == bool else raw_contacts > contact_force_threshold
        return cls(accelerometer, gyroscope, contacts)


@dataclass(frozen=True)
class StabilityEstimate:
    roll: float
    pitch: float
    horizontal_velocity: np.ndarray
    vertical_velocity: float
    contact_count: int
    stable: bool

    @property
    def tilt(self) -> float:
        return float(np.hypot(self.roll, self.pitch))


class StabilityEKF:
    """Small attitude/velocity EKF for early IMU and contact experiments.

    State is [roll, pitch, vx, vy, vz]. Foot contact provides a zero-velocity
    observation; leg geometry can be added later without changing this API.
    """

    def __init__(self, dt: float, gravity: float = GRAVITY):
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.dt = float(dt)
        self.gravity = float(gravity)
        self.x = np.zeros(5, dtype=np.float64)
        self.P = np.diag([0.15, 0.15, 0.25, 0.25, 0.25]) ** 2
        self.process_noise = np.diag([0.015, 0.015, 0.35, 0.35, 0.5]) ** 2
        self.accel_noise = np.eye(3) * 0.35**2
        self.contact_velocity_noise = np.eye(3) * 0.04**2
        self.max_tilt = np.deg2rad(25.0)
        self.max_horizontal_velocity = 0.25

    @staticmethod
    def _rotation_world_from_body(roll: float, pitch: float) -> np.ndarray:
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        return np.array([
            [cos_pitch, sin_roll * sin_pitch, cos_roll * sin_pitch],
            [0.0, cos_roll, -sin_roll],
            [-sin_pitch, sin_roll * cos_pitch, cos_roll * cos_pitch],
        ])

    def _transition(self, state: np.ndarray, sample: StabilitySensorSample) -> np.ndarray:
        next_state = state.copy()
        next_state[0:2] += sample.gyroscope[0:2] * self.dt
        rotation = self._rotation_world_from_body(next_state[0], next_state[1])
        world_acceleration = rotation @ sample.accelerometer
        world_acceleration[2] -= self.gravity
        next_state[2:5] += world_acceleration * self.dt
        return next_state

    def predict(self, sample: StabilitySensorSample) -> None:
        predicted = self._transition(self.x, sample)
        jacobian = np.eye(5)
        epsilon = 1e-6
        for index in range(5):
            perturbed = self.x.copy()
            perturbed[index] += epsilon
            jacobian[:, index] = (self._transition(perturbed, sample) - predicted) / epsilon
        self.x = predicted
        self.P = jacobian @ self.P @ jacobian.T + self.process_noise

    def _update(self, measurement: np.ndarray, expected: np.ndarray, jacobian: np.ndarray, noise: np.ndarray) -> None:
        innovation = measurement - expected
        innovation_covariance = jacobian @ self.P @ jacobian.T + noise
        gain = np.linalg.solve(innovation_covariance, jacobian @ self.P).T
        self.x += gain @ innovation
        identity = np.eye(self.P.shape[0])
        residual = identity - gain @ jacobian
        self.P = residual @ self.P @ residual.T + gain @ noise @ gain.T

    def update_gravity(self, accelerometer: np.ndarray) -> None:
        """Use the accelerometer as a gravity direction when the body is quiet."""
        acceleration_norm = np.linalg.norm(accelerometer)
        if abs(acceleration_norm - self.gravity) > 1.5:
            return

        expected = self._rotation_world_from_body(self.x[0], self.x[1]).T @ np.array([0.0, 0.0, self.gravity])
        jacobian = np.zeros((3, 5))
        epsilon = 1e-6
        for index in (0, 1):
            perturbed = self.x.copy()
            perturbed[index] += epsilon
            perturbed_expected = self._rotation_world_from_body(perturbed[0], perturbed[1]).T @ np.array([0.0, 0.0, self.gravity])
            jacobian[:, index] = (perturbed_expected - expected) / epsilon
        self._update(accelerometer, expected, jacobian, self.accel_noise)

    def update_contact_velocity(self, foot_contacts: np.ndarray) -> None:
        if np.any(foot_contacts):
            self._update(np.zeros(3), self.x[2:5], np.hstack((np.zeros((3, 2)), np.eye(3))), self.contact_velocity_noise)

    def step(self, sample: StabilitySensorSample) -> StabilityEstimate:
        self.predict(sample)
        self.update_gravity(sample.accelerometer)
        self.update_contact_velocity(sample.foot_contacts)
        horizontal_velocity = self.x[2:4].copy()
        return StabilityEstimate(
            roll=float(self.x[0]),
            pitch=float(self.x[1]),
            horizontal_velocity=horizontal_velocity,
            vertical_velocity=float(self.x[4]),
            contact_count=int(np.count_nonzero(sample.foot_contacts)),
            stable=bool(
                np.hypot(self.x[0], self.x[1]) < self.max_tilt
                and np.linalg.norm(horizontal_velocity) < self.max_horizontal_velocity
                and np.count_nonzero(sample.foot_contacts) > 0
            ),
        )

    def step_low_state(self, low_state, contact_force_threshold: float = 20.0) -> StabilityEstimate:
        return self.step(StabilitySensorSample.from_low_state(low_state, contact_force_threshold))

class StateEstimator:
    def __init__(self, dt: float):
        self.dt = dt
        # State [Height, Vertical Velocity]
        self.x = np.array([[0.28], [0.0]]) 
        
        # Covariance matrices
        self.P = np.eye(2) * 0.1     # Uncertainty in state
        self.Q = np.diag([0.01, 0.1]) # Process noise (how much we trust physics)
        self.R = 0.05                 # Measurement noise (how much we trust the "measured_z")
        
    def predict(self, z_accel: float):
        """
        Physics Step: Predict where we are based on acceleration.
        """
        # 1. Remove gravity to get linear acceleration
        a_world = z_accel - 9.81
        
        # 2. State Transition Matrix (Physics: pos = pos + v*dt)
        F = np.array([[1, self.dt], 
                      [0, 1]])
        
        # 3. Control Input Matrix (Physics: pos = 0.5*a*dt^2)
        B = np.array([[0.5 * self.dt**2], 
                      [self.dt]])
        
        # 4. Predict State and Covariance
        self.x = F @ self.x + B * a_world
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, measured_z: float):
        """
        Correction Step: Adjust prediction based on a known height (e.g. from kinematics).
        """
        # H maps state to measurement (we only measure position, index 0)
        H = np.array([[1, 0]])
        
        # Innovation (Difference between measurement and prediction)
        y = measured_z - (H @ self.x)
        
        # Kalman Gain calculation
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T / S
        
        # Update state and covariance
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ H) @ self.P
        
    @property
    def height(self) -> float:
        return float(self.x[0, 0])

    @property
    def vertical_velocity(self) -> float:
        return float(self.x[1, 0])