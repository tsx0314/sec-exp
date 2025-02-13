import time
import csv
import board
import adafruit_mpu6050
import datetime
from serial import Serial
from pynmeagps import NMEAReader

## Initialize I2C and MPU6050
i2c = board.I2C()
time.sleep(1)

# Initialize MPU6050
mpu = adafruit_mpu6050.MPU6050(i2c, 0x68)
mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS

# File Settings
COMMON_FILE_NAME = "readings"

# Offset
mpu_gyro_offsets = [0.039, 0.01, -0.01]
# mpu_acc_offsets = [-0.853, -0.04257, -0.237]

mpu_acc_offsets = [-0.853, -0.04257, -0.117]

# Read Current Time for Filename
current_time = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
file_path = f"{COMMON_FILE_NAME}_{current_time}.csv"

# Initialize CSV File
with open(file_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
        "GPS_Timestamp", "Latitude", "Longitude",
        "Sensor_Timestamp", "Gyro_X", "Gyro_Y", "Gyro_Z",
        "MPU_Acc_X", "MPU_Acc_Y", "MPU_Acc_Z"
    ])

# Initialize GPS Serial Communication
ser = Serial('/dev/ttyAMA0', 115200, timeout=1)

# Function to collect MPU6050 data
def collect_mpu_data():
    timestamp = datetime.datetime.utcnow().str()
    ax, ay, az = mpu.acceleration
    gx, gy, gz = mpu.gyro

    return [
        timestamp,
        gx + mpu_gyro_offsets[0], gy + mpu_gyro_offsets[1], gz + mpu_gyro_offsets[2],
        ax + mpu_acc_offsets[0], ay + mpu_acc_offsets[1], az + mpu_acc_offsets[2]
    ]

# Function to read GPS data
def read_gps(ser):
    try:
        if ser.in_waiting > 0:
            data = ser.readline().decode(errors='ignore').strip()
            # print(f"Raw GPS Data: {data}")  # Debugging line
            if data.startswith('$GNRMC'):
                try:
                    gps_data = NMEAReader.parse(data.encode())  # Ensure bytes format
                    return [gps_data.time.str(), gps_data.lat, gps_data.lon]
                except Exception as e:
                    print(f"GPS Parse Error: {e}")
                    return [None, None, None]
    except Exception as e:
        print(f"GPS Read Error: {e}")

    return [None, None, None]

def main():
    gps_data = [None, None, None]
    print(f"Logging data to {file_path}...")
    try:
        with open(file_path, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            ser.reset_input_buffer()  # Clears input buffer
            time.sleep(0.01)
            while True:
                # Read GPS Data if available
                new_gps_data = read_gps(ser)
                if any(new_gps_data): 
                    gps_data = new_gps_data

                # Read MPU6050 Data (10 samples per 0.1s)
                mpu_batch = []
                i = 0
                while i < 10: 
                    mpu_batch.append(collect_mpu_data())
                    time.sleep(0.0063)
                    i += 1
                # Write 10 MPU readings with the latest GPS reading
                for mpu_data in mpu_batch:
                    full_data = gps_data + mpu_data
                    writer.writerow(full_data)
                    print(f"Data: {full_data}")

    except KeyboardInterrupt:
        print("\nData collection stopped by user.")
        ser.close()
    except Exception as e:
        print(f"\nError: {e}")
        ser.close()

if __name__ == "__main__":
    main()
