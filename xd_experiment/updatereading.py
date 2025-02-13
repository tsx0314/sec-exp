import pandas as pd

file = 'mpu_dji_exp2.csv'

# Load the existing CSV file
csv_file = file 
df = pd.read_csv(csv_file)

# remove timestamp
df.drop(columns=["Timestamp"], inplace=True)

# reformat UTC_Time
df["UTC_Time"] = df["UTC_Time"].astype(str)
df["UTC_Time"] = df["UTC_Time"].str[:1] + ":" + df["UTC_Time"].str[1:3] + ":" + df["UTC_Time"].str[3:5] + ":" + df["UTC_Time"].str[6:]

# conver to decimal degree

def ddm_to_decimal(coord):
    coord = float(coord)  
    degrees = int(coord // 100) 
    minutes = coord % 100  
    decimal = degrees + (minutes / 60) 
    return decimal

df["Latitude"] = df["Latitude"].apply(ddm_to_decimal)
df["Longitude"] = df["Longitude"].apply(ddm_to_decimal)

# Reorder columns
desired_order = ["UTC_Time", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",  "Latitude","Longitude"]
df = df[desired_order]  # Reorder dataframe

# Save the updated CSV
df.to_csv(f'{file})new.csv', index=False)

print(f'New {file} saved')
