import time
import board
import adafruit_adxl34x
import datetime

import os
import csv

###General Setup----------------------------------------------------------------------------------
###
i2c = board.I2C()  # uses board.SCL and board.SDA
time.sleep(1)
# ADXL
adxl = adafruit_adxl34x.ADXL345(i2c,0x68)
adxl.range = adafruit_adxl34x.Range.RANGE_2_G

print(f"Accelerometer Range set to: {adxl.range}")
COMMON_FILE_NAME = "sensor_reading"

# Variables
count = 0  
t_int = 0.01

# Time function
def time_now():
    now = datetime.datetime.utcnow()
    return(now)

# Write collected data to CSV
def mpu_write_to_file(file_path,data):
    with open(file_path, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(data)  # Write a single row
        print(f"Written to file: {data}")

# Collect data from MPU6050
def mpu_collect_data():
    global count
    count += 1

    time = time_now()
    adxl_accel_x,  adxl_accel_y,  adxl_accel_z = adxl.acceleration
    data = [
        count,
        time,
        adxl_accel_x,
        adxl_accel_y,
        adxl_accel_z
    ]
    return data

def main():
    number = input("Enter a number for the file name: ")
    file_path = f'{COMMON_FILE_NAME}_{number}.csv'
    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Count", "Time [UTC]", 
                         "Acc X [m/s^2]", "Acc Y [m/s^2]", "Acc Z [m/s^2]"])

    try:
        while True:
            start_time = time.time()
            data = mpu_collect_data()
            mpu_write_to_file(file_path,data)
            elapsed_time = time.time() - start_time
            time.sleep(max(0, t_int - elapsed_time))  # 100 HZ
    except KeyboardInterrupt:
        print("\nData collection stopped.")

if __name__ == "__main__":
    main()