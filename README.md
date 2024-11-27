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
