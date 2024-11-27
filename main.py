import argparse
from src.simulation import run_simulation
from src.utils import preprocess_udds_data

def main():
    """
    Main entry point for the Kalman Filter simulation project.
    Provides options for preprocessing, simulation, and parameter tuning.
    """
    parser = argparse.ArgumentParser(description="Kalman Filter Simulation with UDDS Dataset")
    parser.add_argument("--preprocess", action="store_true", help="Preprocess the raw UDDS dataset")
    parser.add_argument("--simulate", action="store_true", help="Run Kalman Filter simulation")
    parser.add_argument("--dataset", type=str, default="data/udds_data.csv", help="Path to the UDDS dataset (CSV)")
    parser.add_argument("--raw_dataset", type=str, default="data/raw_udds_data.rtf", help="Path to the raw UDDS dataset")
    parser.add_argument("--output_csv", type=str, default="data/udds_data.csv", help="Path to save the processed UDDS dataset")
    parser.add_argument("--gamma", type=float, default=0.9, help="Robustness parameter for H-Infinity filtering")
    parser.add_argument("--q_scale", type=float, default=0.1, help="Process noise scaling factor")
    parser.add_argument("--r_scale", type=float, default=0.05, help="Measurement noise scaling factor")
    parser.add_argument("--save_plots", action="store_true", help="Save plots to results folder")
    args = parser.parse_args()

    if args.preprocess:
        preprocess_udds_data(args.raw_dataset, args.output_csv)

    if args.simulate:
        run_simulation(
            dataset_path=args.dataset,
            gamma=args.gamma,
            q_scale=args.q_scale,
            r_scale=args.r_scale,
            save_plots=args.save_plots
        )

if __name__ == "__main__":
    main()
