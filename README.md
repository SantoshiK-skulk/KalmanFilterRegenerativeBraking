# KalmanFilterRegenerativeBraking
A Python implementation of an advanced Kalman Filter with H-Infinity filtering for robust state estimation in regenerative braking systems. This repository includes the filter implementation, unit tests, and a simulation using real-world driving cycle data (UDDS). Perfect for researchers and developers in automotive control systems.

# Kalman Filter for UDDS Data Simulation

## Overview
This repository implements a dual Kalman Filter architecture to estimate:
1. Road friction coefficient (\( \mu \)).
2. Vehicle velocity under regenerative braking scenarios.

The **Urban Dynamometer Driving Schedule (UDDS)** dataset is used to simulate and validate the filters.

## Repository Structure
- `data/`: Contains raw and processed UDDS dataset files.
- `src/`: Core implementation of the Kalman Filters and utilities.
- `tests/`: Unit tests for filters and utilities.

## Setup Instructions
1. Clone the repository.
2. Install dependencies:
   ```bash
   pip install -r requirements.txt

How to Use
1. Preprocess the Raw UDDS Dataset
Convert the raw dataset (data/raw_udds_data.rtf) into a structured CSV file:
bash
Copy code
python main.py --preprocess

Input: data/raw_udds_data.rtf.
Output: data/udds_data.csv.
2. Run Simulation with Default Parameters
Run the Kalman Filter simulation using the preprocessed dataset and default parameters:
bash
python main.py --simulate --save_plots
Run Parameter Tuning
python main.py --tune --gamma_range 0.8,1.2,0.1 --q_scale_range 0.05,0.2,0.05 --r_scale_range 0.05,0.2,0.05

Run Kalman Filter simulation using the tuned parameters


Plots: Saved in results/ as velocity_estimation.png and road_friction_estimation.png.
Logs: Metrics such as MAE and RMSE logged in results/simulation.log.
3. Run Simulation with Custom Parameters
Customize γ,q_scale,r_scale\gamma, q\_scale, r\_scaleγ,q_scale,r_scale for specific scenarios:
bash
a. python3 main.py --simulate --gamma 1.0 --q_scale 0.1 --r_scale 0.05 --save_plots
b. 

Parameters:
--gamma: Adjust robustness of the H-Infinity Kalman Filter.
--q_scale: Scale factor for process noise covariance.
--r_scale: Scale factor for measurement noise covariance.
4. Visualize Parameter Tuning
Analyze how different parameter values affect performance using heatmaps:
bash
Copy code
python main.py --tune --gamma_range 0.8,1.2,0.1 --q_scale_range 0.05,0.2,0.05 --r_scale_range 0.05,0.2,0.05 --save_plots

Range Arguments:
--gamma_range: Range for γ\gammaγ as start,end,step.
--q_scale_range: Range for q_scaleq\_scaleq_scale.
--r_scale_range: Range for r_scaler\_scaler_scale.
Generates heatmaps for MAE and RMSE in results/.

Expected Outputs
Simulation Metrics:
Mean Absolute Error (MAE): Tracks average velocity estimation error.
Root Mean Square Error (RMSE): Measures overall prediction accuracy.
Generated Plots:
Velocity Estimation:
True vs estimated velocity plot.
Road Friction Estimation:
Dynamic road friction estimation plot.
Parameter Tuning Heatmaps:
Visualize MAE and RMSE across γ,q_scale,r_scale\gamma, q\_scale, r\_scaleγ,q_scale,r_scale values.
Logs:
Detailed results saved in results/simulation.log.

Example Workflow
Step-by-Step Guide
Preprocess the dataset:
bash
Copy code
python main.py --preprocess


Run simulation with default parameters:
bash
Copy code
python main.py --simulate --save_plots


Run simulation with custom parameters:
bash
Copy code
python main.py --simulate --gamma 1.1 --q_scale 0.15 --r_scale 0.1 --save_plots


Perform parameter tuning:
bash
Copy code
python main.py --tune --gamma_range 0.8,1.2,0.1 --q_scale_range 0.05,0.2,0.05 --r_scale_range 0.05,0.2,0.05 --save_plots



Advanced Options
Unit Testing
Run unit tests to verify the correctness of the Kalman Filters and utilities:
bash
Copy code
pytest tests/

Custom Datasets
Replace the UDDS dataset with your custom dataset. Ensure it follows this structure:
plaintext
Copy code
Test Time (seconds), Target Speed (mph)

Logging
All simulation results (metrics and insights) are logged in results/simulation.log.

Future Enhancements
Incorporate real-world torque and braking data for more realistic simulations.
Add support for highway driving schedules (e.g., HWFET, FTP).
Automate parameter tuning using grid search or Bayesian optimization.
Integrate visualization dashboards for interactive parameter tuning.

License
This project is licensed under the MIT License. See the LICENSE file for details.


=======
# **Kalman Filter for UDDS Data Simulation**

## **Overview**
This repository provides a dual Kalman Filter implementation to:
1. Dynamically estimate the **road friction coefficient (\( \mu \))**.
2. Accurately predict **vehicle velocity** under regenerative braking scenarios.

The project leverages the **Urban Dynamometer Driving Schedule (UDDS)** dataset for validation and simulation.

---

## **Repository Structure**
```plaintext
repo-root/
├── data/
│   ├── udds_data.csv         # Cleaned and preprocessed UDDS dataset (CSV format)
│   ├── raw_udds_data.rtf     # Original UDDS dataset (raw format)
│   ├── drv_cycle_data.mat    # MATLAB drive cycle data (optional)
│
├── src/
│   ├── parameter_kf.py       # Implementation of the ParameterKF class
│   ├── hinfinity_kf.py       # Implementation of the HInfinityKalmanFilterRB class
│   ├── utils.py              # Utility functions for preprocessing and unit conversions
│   ├── simulation.py         # Script for running simulations and visualizations
│
├── tests/
│   ├── test_kf.py            # Unit tests for ParameterKF and HInfinityKalmanFilterRB
│   ├── test_utils.py         # Unit tests for utility functions
│
├── results/                  # Folder for saving plots and logs
│   ├── simulation.log        # Log of simulation results
│   ├── velocity_estimation.png    # True vs Estimated Velocity plot
│   ├── road_friction_estimation.png  # Road friction estimation plot
│
├── main.py                   # Central script to run preprocessing, simulation, and parameter tuning
├── README.md                 # Detailed instructions for using the repository
├── requirements.txt          # Python dependencies
├── LICENSE                   # Licensing information
=======

