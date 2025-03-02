#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>

using namespace std;
using namespace std::chrono;

// ---- GLOBAL VARIABLES ----
std::atomic<bool> running(true);
std::mutex gps_lock;
std::condition_variable gps_cv;

// struct GPSData {
//     string s;
// };

string latest_gps;

// ---- MPU6050 CLASS ----
#define MPU6050_ADDR 0x68
#define ACCEL_REG 0x3B
#define GYRO_REG  0x43

class MPU6050 {
public:
    int fd;
    
    MPU6050() {
        fd = wiringPiI2CSetup(MPU6050_ADDR);
        if (fd == -1) {
            cerr << "MPU6050 Initialization Failed!" << endl;
        }
        wiringPiI2CWriteReg8(fd, 0x6B, 0); // Wake up MPU6050
    }

    vector<int16_t> readRawData(int reg, int count = 6) {
        vector<int16_t> data(count);
        for (int i = 0; i < count; ++i) {
            int16_t high = wiringPiI2CReadReg8(fd, reg + (i * 2));
            int16_t low = wiringPiI2CReadReg8(fd, reg + (i * 2) + 1);
            int16_t value = (high << 8) | low;
            if (value & 0x8000) value = -(65536 - value);
            data[i] = value;
        }
        return data;
    }

    vector<int16_t> getAccelData() {
        return readRawData(ACCEL_REG);
    }

    vector<int16_t> getGyroData() {
        return readRawData(GYRO_REG);
    }
};

// ---- GPS THREAD FUNCTION ----
void readGPS(int serial_fd) {
    string gps_buffer = "";
    char buffer[512];

    while (running) {
        int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            gps_buffer += string(buffer);  

            size_t pos;
            while ((pos = gps_buffer.find("\n")) != string::npos) {
                string line = gps_buffer.substr(0, pos);
                gps_buffer.erase(0, pos + 1); 

                if (line.find("$GNRMC") != string::npos) {
                    // size_t comma1 = line.find(",");
                    // size_t comma2 = line.find(",", comma1 + 1);
                    // size_t comma3 = line.find(",", comma2 + 1);
                    // size_t comma4 = line.find(",", comma3 + 1);

                    // if (comma1 != string::npos && comma2 != string::npos && comma3 != string::npos && comma4 != string::npos) {
                    //     new_gps.time = line.substr(comma1 + 1, comma2 - comma1 - 1);
                    //     new_gps.latitude = line.substr(comma3 + 1, comma4 - comma3 - 1);
                    //     new_gps.longitude = line.substr(comma4 + 1, line.find(",", comma4 + 1) - comma4 - 1);
                    // }

                    {
                        unique_lock<mutex> lock(gps_lock);
                        latest_gps = line;
                    }
                    gps_cv.notify_one();
                }
            }
        }
        this_thread::sleep_for(milliseconds(10));
    }
}

// ---- SENSOR THREAD FUNCTION ----
void readSensor() {
    MPU6050 mpu;
    // ofstream file("sensor_data.csv");
    // file << "Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Latitude,Longitude\n";

    while (running) {
        auto start_time = steady_clock::now();

        vector<int16_t> accel = mpu.getAccelData();
        vector<int16_t> gyro = mpu.getGyroData();
        string gps_data;
        {
            unique_lock<mutex> lock(gps_lock);
            gps_cv.wait_for(lock, milliseconds(5), [] { return !latest_gps.time.empty(); });
            gps_data = latest_gps;
        }

        // Print to console
        cout << accel[0] << ", " << accel[1] << ", " << accel[2] << ", "
             << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << ", "
             << gps_data << endl;

        // // Save to file
        // file << gps_data.time << ","
        //      << accel[0] << "," << accel[1] << "," << accel[2] << ","
        //      << gyro[0] << "," << gyro[1] << "," << gyro[2] << ","
        //      << gps_data.latitude << "," << gps_data.longitude << "\n";

        auto end_time = steady_clock::now();
        auto elapsed_time = duration_cast<milliseconds>(end_time - start_time).count();
        int sleep_time = 10 - elapsed_time;
        if (sleep_time > 0) {
            this_thread::sleep_for(milliseconds(sleep_time));
        }
    }
    // file.close();
}


int main() {
    wiringPiSetup();

    int serial_fd = serialOpen("/dev/serial0", 115200);
    if (serial_fd == -1) {
        cerr << "Failed to open GPS serial port!" << endl;
        return 1;
    }
    cout << "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,GPS_Data" << endl;

    thread gpsThread(readGPS, serial_fd);
    thread sensorThread(readSensor);

    // cout << "Press Enter to stop...\n";
    // cin.get();

    // running = false;

    gpsThread.join();
    sensorThread.join();
    serialClose(serial_fd);

    cout << "Program terminated.\n";
    return 0;
}
