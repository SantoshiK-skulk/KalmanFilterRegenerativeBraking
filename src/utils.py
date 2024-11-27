import pandas as pd

def preprocess_udds_data(input_path, output_path):
    """
    Preprocess the raw UDDS dataset and save it as a CSV file.

    Parameters:
        input_path (str): Path to the raw UDDS dataset (e.g., .rtf or .txt).
        output_path (str): Path to save the cleaned and processed CSV file.
    """
    # Load the raw UDDS data (assume a simple format for parsing)
    data_lines = []
    with open(input_path, "r") as file:
        for line in file:
            if "," in line:  # Ensure valid data lines
                data_lines.append(line.strip().split(","))

    # Create DataFrame
    columns = data_lines[0]
    data_values = data_lines[1:]
    udds_df = pd.DataFrame(data_values, columns=columns)
    udds_df = udds_df.astype({"Test Time": float, "Target Speed": float})  # Convert columns to numeric

    # Process data: Convert speed to m/s and compute acceleration
    udds_df['Speed_mps'] = udds_df['Target Speed'] * 0.44704  # mph to m/s
    udds_df['Acceleration_mps2'] = udds_df['Speed_mps'].diff() / udds_df['Test Time'].diff()
    udds_df['Acceleration_mps2'].fillna(0, inplace=True)  # Handle missing acceleration

    # Save processed data
    udds_df.to_csv(output_path, index=False)
    print(f"Processed UDDS data saved to {output_path}")
