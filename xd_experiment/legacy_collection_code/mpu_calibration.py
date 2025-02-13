import time
import board
import adafruit_mpu6050 
from serial import Serial

import time
import board
import adafruit_mpu6050
import adafruit_adxl34x

i2c = board.I2C()
time.sleep(1)  # Add this delay

mpu1 = adafruit_mpu6050.MPU6050(i2c,0x68)
time.sleep(0.5)  # Allow time for initialization

print("MPU6050 initialized successfully!")

mpu1.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
mpu1.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS
print("MPU initialised.")

# adxl = adafruit_adxl34x.ADXL345(i2c,0x53)
# adxl.range = adafruit_adxl34x.Range.RANGE_2_G
print("ADXL initialised.")

def main():
    count = 0
    sum_gyro_x = sum_gyro_y = sum_gyro_z = 0
    sum_acc_x = sum_acc_y = sum_acc_z = 0
    sum_adxl_x = sum_adxl_y =sum_adxl_z = 0

    while count < 4000:
        gyro_x, gyro_y, gyro_z = mpu1.gyro
        acc_x, acc_y, acc_z = mpu1.acceleration
        # adxl_x, adxl_y, adxl_z = adxl.acceleration
        mpu_acc_offset_x = 0.853
        mpu_acc_offset_y  = 0.042
        mpu_acc_offset_z = 0.137

        acc_x = acc_x - mpu_acc_offset_x
        acc_y = acc_y - mpu_acc_offset_y
        acc_z = acc_z - mpu_acc_offset_z
        # print(f'Gyro: {gyro_x},{gyro_y},{gyro_z}\n')
        # print(f'Acc: {acc_x - 0.08684883988596614 * 9.81},{acc_y - 0.0043466711960142845 * 9.81},{acc_z - 0.044877393082800365 * 9.81}\n')
        # print(f'Acc: {acc_x - 0.08684883988596614 * 9.81},{acc_y - 0.0043466711960142845 * 9.81},{acc_z - 0.015 * 9.81}\n')
        # print(f'Acc_zadxl: {adxl_x},{adxl_y},{adxl_z}\n')
        sum_gyro_x += gyro_x
        sum_gyro_y += gyro_y
        sum_gyro_z += gyro_z

        sum_acc_x += acc_x
        sum_acc_y += acc_y
        sum_acc_z += acc_z

        # sum_adxl_x += adxl_x
        # sum_adxl_y += adxl_y
        # sum_adxl_z += adxl_z

        count += 1
        time.sleep(0.01)  # Adjust sleep time as needed

    print("Sum of Gyroscope Readings:")
    print(f"X: {sum_gyro_x / 4000:.6f}, Y: {sum_gyro_y / 4000:.6f}, Z: {sum_gyro_z / 4000:.6f}")
    print("Sum of Accelerometer Readings:")
    print(f"X: {sum_acc_x / 4000:.6f}, Y: {sum_acc_y / 4000:.6f}, Z: {sum_acc_z / 4000:.6f}")

    # acc - X: 1.022291, Y: -0.098392, Z: 9.978415
    # X: 1.038611, Y: -0.105908, Z: 9.968898
    # gyro - X: -0.039650, Y: -0.012354, Z: 0.011342
    # X: -0.039934, Y: -0.011991, Z: 0.008889


if __name__ == "__main__":
    main()