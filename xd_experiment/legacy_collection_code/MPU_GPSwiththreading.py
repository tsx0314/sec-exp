import time
import csv
import board
import adafruit_mpu6050
import datetime
from serial import Serial
from pynmeagps import NMEAReader
import threading

# Initialize I2C and MPU6050
i2c = board.I2C()
time.sleep(1)

# Initialize MPU6050
mpu = adafruit_mpu6050.MPU6050(i2c, 0x68)
mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS

# File Settings
COMMON_FILE_NAME = "readings"

# Calibration Offsets
mpu_gyro_offsets = [0.039, 0.01, -0.01]
mpu_acc_offsets = [-0.853, -0.04257, -0.117]

current_time = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
file_path = f"{COMMON_FILE_NAME}_{current_time}.csv"

# Initialize CSV File
with open(file_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
        "UTC_Time",
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z",
        "Latitude", "Longitude"
    ])

# GPS Thread Setup
ser = Serial('/dev/ttyAMA0', 115200, timeout=1)
gps_lock = threading.Lock()
latest_gps = {
    "time": None,
    "lat": None,
    "lon": None
}

def gps_reader():
    # ser.reset_input_buffer()
    # ser.reset_output_buffer()
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode(errors='ignore').strip()
                
                if not data.startswith("$GNRMC"):
                    continue 
                
                print(f"Raw GPS Data: {data}")  

                try:
                    msg = NMEAReader.parse(data.encode())
                    if msg.status == "A":  
                        with gps_lock:
                            latest_gps["time"] = msg.time
                            latest_gps["lat"] = msg.lat
                            latest_gps["lon"] = msg.lon
                except Exception as e:
                    print(f"GPS Parsing Error: {e}")
        
        except Exception as e:
            print(f"GPS Reading error: {e}")
        
        time.sleep(0.01)  

# Start GPS Thread
gps_thread = threading.Thread(target=gps_reader, daemon=True)
gps_thread.start()

def collect_sensor_data():
    # timestamp = datetime.datetime.utcnow().time().isoformat()
    ax, ay, az = mpu.acceleration
    gx, gy, gz = mpu.gyro
    return (
       #  timestamp,
        ax + mpu_acc_offsets[0],
        ay + mpu_acc_offsets[1],
        az + mpu_acc_offsets[2],
        gx + mpu_gyro_offsets[0],
        gy + mpu_gyro_offsets[1],
        gz + mpu_gyro_offsets[2]
        ) 

def main():
    print(f"Data Recording: {file_path}")
    try:
        with open(file_path, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            interval = 0.01 
            while True:
                start = time.time()
                # mpu
                sensor_data = collect_sensor_data()
                # gps
                with gps_lock:
                    gps_time = latest_gps["time"]
                    lat = latest_gps["lat"]
                    lon = latest_gps["lon"]             
                # write data
                writer.writerow([
                    gps_time if gps_time else "",
                    sensor_data[0] if sensor_data[0] is not None else "",
                    sensor_data[1] if sensor_data[1] is not None else "",
                    sensor_data[2] if sensor_data[2] is not None else "",
                    sensor_data[3] if sensor_data[3] is not None else "",
                    sensor_data[4] if sensor_data[4] is not None else "",
                    sensor_data[5] if sensor_data[4] is not None else "",
                    lat if lat is not None else "",
                    lon if lon is not None else ""
                ])
                print(f"Data:{gps_time},{datetime.datetime.utcnow()},{lat},{lon}")
                end = time.time()
                usage_time = end - start
                time.sleep(max(0,interval - usage_time))
    except KeyboardInterrupt:
        ser.close()
        print("\nKeyboarding interruption. Stop collection. Serial closed")
    except Exception as e:
        ser.close()
        print(f"\n Error: {str(e)}")

if __name__ == "__main__":
    main()