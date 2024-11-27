from parameter_kf import ParameterKF
from hinfinity_kf import HInfinityKalmanFilterRB
from utils import load_udds_data
import numpy as np
import matplotlib.pyplot as plt

# Load the UDDS dataset
data = load_udds_data('data/udds_data.csv')
velocity = data['velocity']  # Velocity (m/s)
acceleration = data['acceleration']  # Acceleration (m/s^2)
torque = data['torque']  # Regenerative torque (Nm)
time_steps = data['time']  # Time (seconds)

# Parameters
vehicle_mass = 1500  # Vehicle mass in kg
wheel_radius = 0.3  # Wheel radius in meters
g = 9.81  # Gravitational acceleration (m/s^2)

# Compute braking force and road friction coefficient (mu)
braking_force = torque / wheel_radius
effective_mu = braking_force / (vehicle_mass * g)

# Filter valid braking phases (deceleration only)
valid_mu = effective_mu[acceleration < 0]  # Deceleration phases only

# Compute initial_mu
initial_mu = np.mean(valid_mu)
print(f"Dynamic initial_mu calculated from dataset: {initial_mu:.3f}")

# Initialize the ParameterKF (secondary Kalman Filter)
parameter_kf = ParameterKF(
    initial_mu=initial_mu,  # Dynamically computed initial_mu
    acceleration_variance=np.var(acceleration),
    measurement_error_variance=0.05  # Moderate sensor noise
)

# Initialize the HInfinityKalmanFilterRB (primary Kalman Filter)
primary_kf = HInfinityKalmanFilterRB(
    gamma=0.9,  # Robustness parameter
    mass=vehicle_mass,  # Vehicle mass
    wheel_radius=wheel_radius,  # Wheel radius
    time_step=np.mean(np.diff(time_steps)),  # Average time step from dataset
    mu=initial_mu,  # Use dynamically computed initial_mu
    alpha=0.95,  # Torque decay factor
    q_scale=0.1,  # Process noise scaling factor
    r_scale=0.05  # Measurement noise scaling factor
)

# Simulate the dual Kalman Filter system
estimated_mu_values = []  # Store estimated road friction (mu)
estimated_states = []  # Store estimated velocity and torque

for t in range(len(velocity)):
    # Simulate observed road friction with some noise
    observed_friction = initial_mu + np.random.normal(0, 0.05)  # Noisy observations

    # Secondary KF: Predict and update road friction
    parameter_kf.predict()
    parameter_kf.update(observed_friction)
    estimated_mu = parameter_kf.get_parameter()
    estimated_mu_values.append(estimated_mu)

    # Primary KF: Predict and update state (velocity, torque)
    primary_kf.predict(road_friction=estimated_mu)
    primary_kf.update(np.array([[velocity[t]], [torque[t]]]))
    estimated_states.append(primary_kf.get_state())

    # Print results for each time step
    print(f"Time {t}:")
    print(f"  Observed Road Friction: {observed_friction:.3f}")
    print(f"  Estimated Road Friction (mu): {estimated_mu:.3f}")
    print(f"  Estimated State: {primary_kf.get_state()}")

# Visualization: Road friction estimation
plt.figure(figsize=(10, 6))
plt.plot(time_steps, estimated_mu_values, label="Estimated Road Friction (mu)")
plt.axhline(initial_mu, color="red", linestyle="--", label="True Initial Friction")
plt.xlabel("Time (s)")
plt.ylabel("Road Friction (mu)")
plt.title("Road Friction Estimation Over Time")
plt.legend()
plt.show()

# Visualization: Velocity estimation
velocity_estimates = np.array([state[0] for state in estimated_states])  # Extract velocity estimates

plt.figure(figsize=(10, 6))
plt.plot(time_steps, velocity, label="True Velocity", color="blue")
plt.plot(time_steps, velocity_estimates, label="Estimated Velocity", color="orange", linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("True vs Estimated Velocity Over Time")
plt.legend()
plt.show()
