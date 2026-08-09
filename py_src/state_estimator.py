import numpy as np

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