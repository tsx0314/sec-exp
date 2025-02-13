#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <string>
#include <sstream>
#include <cmath>  
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <Eigen/Dense>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <condition_variable>

#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

#define SERIAL_PORT "/dev/ttyAMA0"
#define BAUDRATE B115200
constexpr double STANDARD_GRAVITY = 9.80665;
constexpr double gyro_scale = 131;
using namespace std;
using namespace std::chrono;

vector<double>mpu_gyro_offsets = {0.039, 0.01, -0.01};
vector<double> mpu_acc_offsets = {-0.853, -0.04257, -0.117};

// Global Variables
mutex gps_lock;
condition_variable gps_cv;
int serial_fd;

struct GPSData {
    string time;
    string latitude = "";
    string longitude = "";
};

struct SensorData {
    long double acc_x = 0.0;
    long double acc_y = 0.0;
    long double acc_z = 0.0;
    long double gyro_x = 0.0;
    long double gyro_y = 0.0;
    long double gyro_z = 0.0;
};

GPSData latest_gps{ "nan", "nan", "nan" };
SensorData latest_sensor;
bool new_gps_available = false;

int init_serial() {
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        cerr << "Error opening serial port!" << endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd, &tty) != 0) {
        cerr << "Error from tcgetattr" << endl;
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);
    
    tty.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tcflush(serial_fd, TCIFLUSH);
    tcsetattr(serial_fd, TCSANOW, &tty);
    return serial_fd;
}

void gps_reader(int serial_fd) {
    string gps_buffer = "";  
    char buffer[512];

    while (true) {
        int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            gps_buffer += string(buffer);  // Append to buffer

            size_t pos;
            while ((pos = gps_buffer.find("\n")) != string::npos) {
                string line = gps_buffer.substr(0, pos);
                gps_buffer.erase(0, pos + 1);  // Remove processed line
                // cout << line << endl;
                if (line.find("$GNRMC") != string::npos) {
                    istringstream ss(line);
                    vector<string> tokens;
                    string token;
                    while (getline(ss, token, ',')) tokens.push_back(token);

                    if (tokens.size() > 6 && tokens[2] == "A") {
                        lock_guard<mutex> lock(gps_lock);
                        latest_gps.time = tokens[1];
                        latest_gps.latitude = tokens[3];
                        latest_gps.longitude = tokens[5];
                        new_gps_available = true;
                        gps_cv.notify_all();
                    }
                }
            }
        }
        //this_thread::sleep_for(milliseconds(10));  
    }
}

// MPU6050 mpu(0x68);
class MPU6050 {
    public:
        int fd;  // File descriptor for I2C device
    
        MPU6050() {
            fd = wiringPiI2CSetup(MPU6050_ADDR);  // Initialize I2C
            wiringPiI2CWriteReg8(fd, 0x6B, 0);   // Wake up MPU6050
        }
    
        std::vector<int16_t> read_sensor(int reg, int count = 6) {
            std::vector<int16_t> data(count);
            for (int i = 0; i < count; ++i) {
                int16_t value = (wiringPiI2CReadReg8(fd, reg + (i * 2)) << 8) | 
                                wiringPiI2CReadReg8(fd, reg + (i * 2) + 1);
    
                if (value & 0x8000)  
                    value = -(65536 - value);
    
                // data[i] = value * (1 / 16384.0) * STANDARD_GRAVITY + mpu_acc_offsets[i];  
                data[i] = value;
            }
    
            return data;
        }
        // std::vector<long double> read_gyro(int reg, int count = 3) {
        //     std::vector<long double> data(count);
        //     for (int i = 0; i < count; ++i) {
        //         int16_t value = (wiringPiI2CReadReg8(fd, reg + (i * 2)) << 8) | 
        //                         wiringPiI2CReadReg8(fd, reg + (i * 2) + 1);
    
        //         if (value & 0x8000) {
        //             value = -(65536 - value);
        //         } 
        //         data = value;
        //         // data[i] = (value / gyro_scale) * (M_PI / 180.0) + mpu_gyro_offsets[i];
                  
        //     }
    
        //     return data;
        // }
    };

void main_thread() {
    MPU6050 mpu;
    // string file_name = "readings_" + to_string(time(nullptr)) + ".csv";
    ofstream file(file_name);
    // file << "UTC_Time,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,Latitude,Longitude\n";

    // auto next_read = steady_clock::now();

    while (true) {
        auto start_time = steady_clock::now();
        vector<autp> accel = mpu.read_acc(ACCEL_XOUT_H);  // Read accelerometer data
        vector<auto> gyro = mpu.read_gyro(GYRO_XOUT_H);   // Read gyroscope data
        vector<auto> sensor_data = accel;
       sensor_data.insert(sensor_data.end(), gyro.begin(), gyro.end());
        GPSData gps_data;
        {
            unique_lock<mutex> lock(gps_lock);
            if (gps_cv.wait_for(lock, milliseconds(1), [] { return !latest_gps.time.empty(); })) {
                gps_data = latest_gps;
            }
        }
        
        auto now = system_clock::now();  
        time_t now_c = system_clock::to_time_t(now);  
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;  
    
        // file << gps_data.time << ",";
        // for (long double val : sensor_data) file << val << ",";
        // file << gps_data.latitude << "," << gps_data.longitude << ", "
        //     << put_time(localtime(&now_c), "%Y-%m-%d %H:%M:%S") 
        //     << '.' << setfill('0') << setw(3) << ms.count()  
        //      << "\n";

        auto end_time = steady_clock::now();
        auto elapsed_time = duration_cast<milliseconds>(end_time - start_time).count();
        auto sleep_time = 10 - elapsed_time;  
        if (sleep_time > 0) {
            this_thread::sleep_for(milliseconds(sleep_time));
        }

        // cout << "Data recorded: " << gps_data.time << "|" ;
        cout << gps_data.time << ",";
        for (long double val : sensor_data) cout<< val << ",";
        cout << gps_data.latitude << "," << gps_data.longitude << "\n";
        // next_read += milliseconds(5);  
    }
}


int main() {
    if (init_serial() == -1) {
        cerr << "Failed to initialize serial port." << endl;
        return 1;
    }

    thread gps_worker(gps_reader, serial_fd);
    thread main_worker(main_thread);

    gps_worker.join();
    main_worker.join();

    close(serial_fd);
    return 0;
}
