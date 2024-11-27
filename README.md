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
## **Setup Instructions**

### **1. Clone the Repository**
To get started, clone the repository to your local machine:
```bash
git clone https://github.com/your-username/kalman-filter-udds.git
cd kalman-filter-udds

### **2. Install Python Dependencies
Ensure you have Python 3.8+ installed. Install the required packages:

```bash
pip install -r requirements.txt
