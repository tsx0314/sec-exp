###########################
#        MPU & ADXL       #
###########################

##Imports------------------------------------------------------------------------------------
# Import libraries
import time
import board
import adafruit_adxl34x
import adafruit_mpu6050 
import datetime

import os
import csv

###General Setup----------------------------------------------------------------------------------
###
i2c = board.I2C()  # uses board.SCL and board.SDA
time.sleep(1)
#MPU
mpu = adafruit_mpu6050.MPU6050(i2c)
mpu.accelerometer_range = 0x00
gy_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS

#ADXL
adxl = adafruit_adxl34x.ADXL345(i2c)
adxl.range = adafruit_adxl34x.Range.RANGE_2_G

FILE_PATH = "sensor_reading.csv"

AccelMinX= -9.92  
AccelMaxX = 11.10
AccelMinY = -10.28  
AccelMaxY = 10.98
AccelMinZ = -10.67  
AccelMaxZ = 9.92

# Input
measur_no, acc_range_in, gy_range_in = input(
    "Enter Measurement number, Accelerometer range (+/- 2, 4, 8, 16 g), and Gyro range (+/- 250, 500, 1000, 2000 °), separated by spaces: "
).split()

#Variables
count = 1           #count variable to sort measurements
t_int = 0.01        #define interval T = 1/f, where f = sampling rate/frqn [Hz]
t_corr = 0.00821    #correction factor for programm run time

# ##Functions------------------------------------------------------------------------------------
#Time function
def time_now():
    now = datetime.datetime.now(datetime.timezone.utc)
    print(f"Timestamp :{now}")
    return(now)

# def write_to_csv(file_path, measur_no, time, mpu_gx, mpu_gy, mpu_gz, adxl_x, adxl_y, adxl_z):
#     try:
#         # Open the file in append mode
#         with open(f"{file_path}.csv", mode='a', newline='') as sensor_readings:
#             sensor_write = csv.writer(sensor_readings, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#             sensor_write.writerow([measur_no, time, mpu_gx, mpu_gy, mpu_gz, adxl_x, adxl_y, adxl_z])
#             print("Data written successfully.")
            
#     except Exception as e:
#         print(f"An error occurred: {e}")

##Sensor settings-----------------------------------------------------------------------------
#MPU
#Set Acceleration Range of MPU
#2G = 0x00, 4G = 0x08, 8G = 0x10, 16G = 0x18

#set inital value for Accelerometer range
mpu.accelerometer_range = 0x00

#Condition acc
# Mapping using the constants
if acc_range_in == str(2):  
    mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
elif acc_range_in == str(4):  
    mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_4_G
elif acc_range_in == str(8): 
    mpu.accelerometer_range= adafruit_mpu6050.Range.RANGE_8_G
elif acc_range_in == str(16):  
    mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_16_G
else:
    print("Invalid input!")

# Displaying the result
print(f"Accelerometer Range set to: {mpu.accelerometer_range}")

#Set Gyro Range
#250DEG = 0x00, 500DEG = 0x08, 1000DEG = 0x10, 2000DEG = 0x18

#set inital value for Gyro range
gy_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS

#Condition
if gy_range_in == str(250):
    mpu.gy_range = 0x00
elif gy_range_in == str(500):
    mpu.gy_range = 0x08
elif gy_range_in == str(1000):
    mpu.gy_range = 0x10
elif gy_range_in == str(2000):
    mpu.gy_range = 0x18
else:
    print("something went wrong!")
# Displaying the result
print(f"Gyro Range set to: {mpu.gy_range}")
time.sleep(1)
#ADXL
#Set Rate of ADXL
adxl.data_rate = adafruit_adxl34x.DataRate.RATE_1600_HZ
adxl.range = adafruit_adxl34x.Range.RANGE_2_G

#Condition
if acc_range_in == str(2):
    adxl.range = adafruit_adxl34x.Range.RANGE_2_G
elif acc_range_in == str(4):
    adxl.range = adafruit_adxl34x.Range.RANGE_4_G
elif acc_range_in == str(8):
    adxl.range = adafruit_adxl34x.Range.RANGE_8_G
elif acc_range_in == str(16):
    adxl.range = adafruit_adxl34x.Range.RANGE_16_G
else:
    print("something went wrong!")
print(f"Gyro Range set to: {adxl.range}")

##Initialization-------------------------------------------------------------------------------
#First row of csv
# write_to_csv(FILE_PATH, "count","time [UTC]","mpu gyro x [°]","mpu gyro y [°]","mpu gyro z [°]","adxl acc x [m/s^2]","adxl acc y [m/s^2]","adxl acc z [m/s^2]","adxl calibrated x","adxl calibrated y","adxl calibrated z" )
with open(FILE_PATH, 'a', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(["Count","time [UTC]","mpu gyro x [°]","mpu gyro y [°]","mpu gyro z [°]","adxl acc x [m/s^2]","adxl acc y [m/s^2]","adxl acc z [m/s^2]","adxl calibrated x","adxl calibrated y","adxl calibrated z"])

#Test running time of function
# start=time.time()
#------
# mpu_accel_x, mpu_accel_y,mpu_accel_z = mpu.acceleration
# mpu_gyro_x, mpu_gyro_y,mpu_gyro_z = mpu.gyro
# adxl_x, adxl_y, adxl_z  = adxl.acceleration
# # calibrated_x = (adxl_x - ((AccelMinX + AccelMaxX) / 2)) 
# # calibrated_y = (adxl_y - ((AccelMinY + AccelMaxY) / 2)) 
# # calibrated_z = (adxl_z - ((AccelMinZ + AccelMaxZ) / 2)) 
# write_to_csv(FILE_PATH, 1, time_now(), mpu_accel_x, mpu_accel_y, mpu_accel_z, mpu_gyro_x, mpu_gyro_y, mpu_gyro_z, adxl_x, adxl_y, adxl_z, 0, 0,0)
# # count+=1
# data_to_write = []
# batch_size = 10  # Number of batches you want to write

# # Accumulate all data for 20 batches
# for _ in range(batch_size):
#     mpu_accel_x, mpu_accel_y, mpu_accel_z = mpu.acceleration
#     mpu_gyro_x, mpu_gyro_y, mpu_gyro_z = mpu.gyro
#     adxl_x, adxl_y, adxl_z = adxl.acceleration
#     data_to_write.append([time_now(),mpu_gyro_x, mpu_gyro_y, mpu_gyro_z, adxl_x, adxl_y, adxl_z])

# # Write all the data at once into the CSV file
# with open(FILE_PATH, 'a', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerows(data_to_write)

# #------
# end=time.time()
# run_time = end - start
# print("Program run time: " + str(run_time) +" s")
# time.sleep(10)

##Collect data and write into csv---------------------------------------------------------------
#While loop that subtracts running time of program to get the desired Hz-rate
count = 1
try:
    while True:
        # Fetch sensor data
        # mpu_accel_x, mpu_accel_y, mpu_accel_z = mpu.acceleration
        # mpu_gyro_x, mpu_gyro_y, mpu_gyro_z = mpu.gyro
        # adxl_x, adxl_y, adxl_z = adxl.acceleration

        # calibrated_x = (adxl_x - ((AccelMinX + AccelMaxX) / 2)) 
        # calibrated_y = (adxl_y - ((AccelMinY + AccelMaxY) / 2)) 
        # calibrated_z = (adxl_z - ((AccelMinZ + AccelMaxZ) / 2)) 

        # # # Print the values
        # print(f"MPU6050 Acceleration: X={mpu_accel_x:.2f} m/s^2, Y={mpu_accel_y:.2f} m/s^2, Z={mpu_accel_z:.2f} m/s^2")
        # print(f"MPU6050 Gyroscope:    X={mpu_gyro_x:.2f} °/s, Y={mpu_gyro_y:.2f} °/s, Z={mpu_gyro_z:.2f} °/s")
        # print(f"Actual Accel (g): X={calibrated_x:.3f} Y={ calibrated_y:.3f} Z={calibrated_z:.3f}")
        # print("-" * 50) 

        # # Write data to CSV
        # write_to_csv(FILE_PATH, count, time_now(), mpu_accel_x, mpu_accel_y, mpu_accel_z, mpu_gyro_x, mpu_gyro_y, mpu_gyro_z, adxl_x, adxl_y, adxl_z, calibrated_x, calibrated_y,calibrated_z)
        
        # # Increment count
        # count += 1

        # # Pause for the next reading
        # # time.sleep(t_int - t_corr)  # Assuming t_int and t_corr are defined elsewhere
        data_to_write = []
        batch_size = 20  # Number of batches you want to write

        # Accumulate all data for 20 batches
        for _ in range(batch_size):
            mpu_accel_x, mpu_accel_y, mpu_accel_z = mpu.acceleration
            mpu_gyro_x, mpu_gyro_y, mpu_gyro_z = mpu.gyro
            adxl_x, adxl_y, adxl_z = adxl.acceleration
            data_to_write.append([count,time_now(),mpu_gyro_x, mpu_gyro_y, mpu_gyro_z, adxl_x, adxl_y, adxl_z])
            count += 1
        # Write all the data at once into the CSV file
        with open(FILE_PATH, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(data_to_write)
except KeyboardInterrupt:
    print("\nKeyboard Interrupt detected. Exiting...")
    
