import pandas as pd
import numpy as np

file = 'sensor_data_2025-03-05_14-22-49.csv'

# Load the CSV file and print column names
df = pd.read_csv(file)

# Print column names to debug
print("Columns in CSV:", df.columns)

# Strip any leading/trailing spaces in column names
df.columns = df.columns.str.strip()

# Remove Timestamp column if it exists
# if "Timestamp" in df.columns:
#     df.drop(columns=["Timestamp"], inplace=True)

# # Reformat UTC_Time without dropping leading zeros
df["UTC_Time"] = df["UTC_Time"].astype(str)
df["UTC_Time"] = (
    df["UTC_Time"].str[0] + ":" +  # Hours
    df["UTC_Time"].str[1:3] + ":" +  # Minutes
    df["UTC_Time"].str[3:5] + ":" +  # Seconds
    df["UTC_Time"].str[6:].str.replace(".", "")  # Milliseconds without a decimal
)

def ddm_to_decimal(coord):
    if pd.isna(coord):  # Check for NaN
        return np.nan
    try:
        coord = float(coord)  # Convert to float
        degrees = int(coord // 100)  # Extract degrees
        minutes = coord % 100  # Extract minutes
        decimal = degrees + (minutes / 60)  # Convert to decimal degrees
        return round(decimal, 10)  # Round to 10 decimal places
    except ValueError:  # Handle conversion errors
        return np.nan

# Apply conversion to Latitude and Longitude safely
df["Latitude"] = df["Latitude"].apply(ddm_to_decimal)
df["Longitude"] = df["Longitude"].apply(ddm_to_decimal)

# # Convert Latitude and Longitude to Radians
# df["Latitude"] = np.radians(df["Latitude"])
# df["Longitude"] = np.radians(df["Longitude"])

# # Reorder columns
desired_order = ["UTC_Time", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "Latitude", "Longitude", "Timestamp"]
df = df[desired_order]

# Save the updated CSV
output_file = f"{file}_new.csv"
df.to_csv(output_file, index=False)

print(f'New file saved as {output_file}')
