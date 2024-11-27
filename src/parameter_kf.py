import numpy as np

class ParameterKF:
    def __init__(self, initial_mu=0.8, acceleration_variance=0.02, measurement_error_variance=0.05):
        """
        Secondary Kalman Filter for estimating road friction (\mu) using the UDDS dataset.

        Parameters:
            initial_mu: float
                Initial estimate of road friction (\mu).
            acceleration_variance: float
                Variance of acceleration data in the UDDS dataset.
            measurement_error_variance: float
                Variance of measurement noise.
        """
        self.x = np.array([[initial_mu]])  # Initial estimate of road friction
        self.A = np.array([[1.0]])  # Assume constant friction
        self.H = np.array([[1.0]])  # Direct measurement of road friction
        self.Q = np.array([[acceleration_variance * 0.01]])  # Process noise (1% of acceleration variance)
        self.R = np.array([[measurement_error_variance]])  # Measurement noise
        self.P = np.array([[0.01]])  # Initial uncertainty in estimate

    def predict(self):
        """
        Predict step for the parameter Kalman Filter.
        """
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

    def update(self, z):
        """
        Update step for the parameter Kalman Filter.

        Parameters:
            z: float
                Observed road friction value.
        """
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        self.x += np.dot(K, y)
        self.P = (1 - K) * self.P

    def get_parameter(self):
        """
        Get the current parameter estimate.

        Returns:
            float: Estimated road friction.
        """
        return self.x.flatten()[0]
