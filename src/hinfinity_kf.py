import numpy as np

class HInfinityKalmanFilterRB:
    def __init__(self, gamma=1.0, mass=1500, wheel_radius=0.3, time_step=0.1, mu=0.8, alpha=0.95, q_scale=0.1, r_scale=0.05):
        """
        Initialize the H-Infinity Kalman Filter for regenerative braking.

        Parameters:
            gamma: float
                Robustness parameter for H-Infinity filtering.
            mass: float
                Vehicle mass (kg).
            wheel_radius: float
                Effective wheel radius (m).
            time_step: float
                Time step for discrete dynamics (s).
            mu: float
                Initial road friction coefficient.
            alpha: float
                Torque decay factor.
            q_scale: float
                Scaling factor for process noise covariance (Q).
            r_scale: float
                Scaling factor for measurement noise covariance (R).
        """
        self.gamma = gamma
        self.m = mass
        self.r = wheel_radius
        self.dt = time_step
        self.mu = mu  # Road friction coefficient
        self.alpha = alpha  # Torque decay factor
        self.q_scale = q_scale  # Scaling factor for Q
        self.r_scale = r_scale  # Scaling factor for R
        self.x = np.array([[0.0], [0.0]])  # Initial state: [velocity, torque]
        self.A = np.zeros((2, 2))  # State transition matrix (dynamically updated)
        self.H = np.eye(2)  # Measurement matrix
        self.Q = np.array([[1e-4, 0.0], [0.0, 1e-4]])  # Process noise covariance
        self.R = np.array([[0.1, 0.0], [0.0, 0.1]])  # Measurement noise covariance
        self.P = np.eye(2)  # Initial estimate covariance

    def update_state_transition_matrix(self):
        """
        Dynamically update the state transition matrix A based on road friction and torque decay.
        """
        self.A[0, 0] = 1.0
        self.A[0, 1] = -self.mu * self.dt / (self.m * self.r)  # Velocity depends on torque
        self.A[1, 0] = 0.0
        self.A[1, 1] = self.alpha  # Torque decays over time

    def predict(self, road_friction=None):
        """
        Predict step of the Kalman Filter.

        Parameters:
            road_friction: float (optional)
                Updated road friction value. If provided, it updates the filter's road friction.
        """
        if road_friction is not None:
            self.mu = road_friction  # Update road friction dynamically
        self.update_state_transition_matrix()
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

    def update(self, z):
        """
        Update step of the Kalman Filter.

        Parameters:
            z: numpy.ndarray
                Measurement vector.
        """
        y = z - np.dot(self.H, self.x)  # Measurement residual
        self.adapt_covariance(y)  # Adjust covariances dynamically
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S))) * (1 / self.gamma)
        self.x += np.dot(K, y)
        I = np.eye(self.H.shape[0])
        self.P = (I - np.dot(K, self.H)) @ self.P * self.gamma

    def adapt_covariance(self, residual):
        """
        Dynamically adjust Q and R based on the residual.

        Parameters:
            residual: numpy.ndarray
                Measurement residual.
        """
        self.Q *= (1 + np.mean(np.abs(residual)) * self.q_scale)
        self.R *= (1 + np.mean(np.abs(residual)) * self.r_scale)

    def get_state(self):
        """
        Get the current state estimate.

        Returns:
            numpy.ndarray: The estimated state vector.
        """
        return self.x.flatten()
