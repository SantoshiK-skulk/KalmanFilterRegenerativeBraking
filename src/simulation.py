import os
import logging
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import mean_absolute_error, mean_squared_error
from src.parameter_kf import ParameterKF
from src.hinfinity_kf import HInfinityKalmanFilterRB

# Configure logging
logging.basicConfig(
    filename="results/simulation.log",
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)

def run_simulation(dataset_path, gamma=0.9, q_scale=0.1, r_scale=0.05, save_plots=False):
    """
    Run the Kalman Filter simulation using the preprocessed UDDS dataset.

    Parameters:
        dataset_path (str): Path to the preprocessed UDDS dataset (CSV format).
        gamma (float): Robustness parameter for H-Infinity filtering.
        q_scale (float): Process noise scaling factor.
        r_scale (float): Measurement noise scaling factor.
        save_plots (bool): If True, saves the plots to the results folder.
    """
    # Ensure the results folder exists
    os.makedirs("results", exist_ok=True)

    # Load the preprocessed dataset
    logging.info(f"Loading dataset from {dataset_path}...")
    data = pd.read_csv(dataset_path)
    velocity = data['Speed_mps'].values
    acceleration = data['Acceleration_mps2'].values
    time_steps = data['Test Time'].values

    # Mock torque data (simplified calculation proportional to acceleration)
    torque = -acceleration * 1500 * 0.3  # Torque in Nm

    # Vehicle and environment parameters
    vehicle_mass = 1500  # kg
    wheel_radius = 0.3  # meters
    g = 9.81  # m/s²

    # Compute braking force and initial road friction estimate (mu)
    braking_force = torque / wheel_radius
    effective_mu = braking_force / (vehicle_mass * g)
    valid_mu = effective_mu[acceleration < 0]  # Use deceleration phases
    initial_mu = np.mean(valid_mu)
    logging.info(f"Initial road friction estimate (mu): {initial_mu:.3f}")

    # Initialize the filters
    parameter_kf = ParameterKF(
        initial_mu=initial_mu,
        acceleration_variance=np.var(acceleration),
        measurement_error_variance=0.05
    )
    primary_kf = HInfinityKalmanFilterRB(
        gamma=gamma,
        mass=vehicle_mass,
        wheel_radius=wheel_radius,
        time_step=np.mean(np.diff(time_steps)),
        mu=initial_mu,
        alpha=0.95,
        q_scale=q_scale,
        r_scale=r_scale
    )

    # Run simulation
    estimated_mu_values = []
    estimated_states = []

    for t in range(len(velocity)):
        observed_friction = initial_mu + np.random.normal(0, 0.05)
        parameter_kf.predict()
        parameter_kf.update(observed_friction)
        estimated_mu = parameter_kf.get_parameter()
        estimated_mu_values.append(estimated_mu)

        primary_kf.predict(road_friction=estimated_mu)
        primary_kf.update(np.array([[velocity[t]], [torque[t]]]))
        estimated_states.append(primary_kf.get_state())

    # Extract velocity estimates
    velocity_estimates = np.array([state[0] for state in estimated_states])

    # Compute validation metrics
    mae_velocity = mean_absolute_error(velocity, velocity_estimates)
    rmse_velocity = np.sqrt(mean_squared_error(velocity, velocity_estimates))
    logging.info(f"Velocity MAE: {mae_velocity:.3f} m/s")
    logging.info(f"Velocity RMSE: {rmse_velocity:.3f} m/s")

    # Plot results
    # Visualization: Velocity estimation
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, velocity, label="True Velocity", color="blue")
    plt.plot(time_steps, velocity_estimates, label="Estimated Velocity", color="orange", linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("True vs Estimated Velocity Over Time (UDDS Data)")
    plt.legend()
    if save_plots:
        plt.savefig("results/velocity_estimation.png")
    plt.show()

    # Visualization: Road friction estimation
    plt.figure(figsize=(10, 6))
    plt.plot(time_steps, estimated_mu_values, label="Estimated Road Friction (mu)")
    plt.axhline(initial_mu, color="red", linestyle="--", label="True Initial Friction")
    plt.xlabel("Time (s)")
    plt.ylabel("Road Friction (mu)")
    plt.title("Road Friction Estimation Over Time (UDDS Data)")
    plt.legend()
    if save_plots:
        plt.savefig("results/road_friction_estimation.png")
    plt.show()
