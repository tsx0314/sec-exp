import time
import numpy as np
import csv
import datetime
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from smbus2 import SMBus

# I2C Address of MPU6050
MPU6050_ADDR = 0x68

# MPU6050 Register Addresses
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG = 0x1C

# Initialize I2C bus
bus = SMBus(1)

def initialise_mpu():
    """ Initializes the MPU6050 sensor """
    try:
        # Wake up MPU6050
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)  # Set to ±2g range
        time.sleep(0.1)
        print("MPU6050 Initialized Successfully!")
    except Exception as e:
        print("ERROR: MPU6050 not detected! Check wiring.")
        print(e)
        exit()

def read_raw_data(addr):
    """ Reads raw 16-bit value from the given register address """
    try:
        high = bus.read_byte_data(MPU6050_ADDR, addr)
        low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
        value = (high << 8) | low
        if value >= 32768:  # Convert to signed 16-bit
            value -= 65536
        return value
    except Exception as e:
        print("I2C Read Error:", e)
        return 0  # Return 0 in case of failure

def get_accel():
    """ Reads acceleration data from MPU6050 and returns X, Y, Z values in 'g' units """
    accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    accel_y = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    return accel_x, accel_y, accel_z

def accel_fit(x_input, m_x, b):
    """ Fit equation for accelerometer calibration """
    return (m_x * x_input) + b

def accel_cal(cal_size=1000):
    """ Calibrates the accelerometer using gravity """
    print("-" * 50)
    print("Accelerometer Calibration")
    
    mpu_offsets = [[], [], []]  # Offset array to store calibration values
    axis_vec = ['z', 'y', 'x']
    cal_directions = ["upward", "downward", "perpendicular to gravity"]
    cal_indices = [2, 1, 0]  # Corresponding axis indices
    
    for qq, ax_qq in enumerate(axis_vec):
        ax_offsets = [[], [], []]
        print("-" * 50)

        for direc_ii, direc in enumerate(cal_directions):
            input("-" * 8 + f" Press Enter and Keep MPU6050 Steady to Calibrate {ax_qq}-axis {direc}")

            # Clear buffer before taking readings
            [get_accel() for _ in range(0, cal_size)]
            mpu_array = []

            while len(mpu_array) < cal_size:
                try:
                    ax, ay, az = get_accel()
                    mpu_array.append([ax, ay, az])  # Append readings
                except:
                    continue

            ax_offsets[direc_ii] = np.array(mpu_array)[:, cal_indices[qq]]  # Store offset values

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts, _ = curve_fit(
            accel_fit, 
            np.append(np.append(ax_offsets[0], ax_offsets[1]), ax_offsets[2]),
            np.append(np.append(1.0 * np.ones(np.shape(ax_offsets[0])),
                                -1.0 * np.ones(np.shape(ax_offsets[1]))),
                      0.0 * np.ones(np.shape(ax_offsets[2]))),
            maxfev=10000
        )

        mpu_offsets[cal_indices[qq]] = popts  # Store slope and intercept
        print(f"Calibration for {ax_qq}-axis:")
        print(f"  - Slope: {popts[0]}")
        print(f"  - Offset: {popts[1]}")

    print("Accelerometer Calibration Complete")
    return mpu_offsets

if __name__ == '__main__':
    initialise_mpu()

    # Calibrate Accelerometer
    cal_size = 1000  # Number of points for calibration
    accel_coeffs = accel_cal(cal_size)

    # Record new acceleration data
    data = np.array([get_accel() for _ in range(0, cal_size)])

    # Plot with and without offsets
    plt.style.use('ggplot')
    fig, axs = plt.subplots(2, 1, figsize=(12, 9))

    accel_labels = ['a_x', 'a_y', 'a_z']
    
    for ii in range(3):
        axs[0].plot(data[:, ii], label=f'${accel_labels[ii]}$, Uncalibrated')
        axs[1].plot(accel_fit(data[:, ii], *accel_coeffs[ii]), label=f'${accel_labels[ii]}$, Calibrated')

    axs[0].legend(fontsize=14)
    axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$a_{x,y,z}$ [g]', fontsize=18)
    axs[1].set_ylabel('$a_{x,y,z}$ [g]', fontsize=18)
    axs[1].set_xlabel('Sample', fontsize=18)
    axs[0].set_ylim([-2, 2])
    axs[1].set_ylim([-2, 2])
    axs[0].set_title('Accelerometer Calibration Correction', fontsize=18)

    fig.savefig('accel_calibration_output.png', dpi=300, bbox_inches='tight', facecolor='#FCFCFC')
    plt.show()
