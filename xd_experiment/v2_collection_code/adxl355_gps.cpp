#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <chrono>
#include <atomic>
#include <vector>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include "wiringSerial.h"
#include <cstring>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace std::chrono;

// Shared variables
atomic<bool> running(true);
mutex gps_lock, data_lock;
condition_variable gps_cv;

// Holds latest GPS time, latitude, longitude
string latest_gps_time = "";
string latest_latitude = "";
string latest_longitude = "";

// GPS Data Structure
struct AccData {
    float acc_x;
    float acc_y;
    float acc_z;
    string time;
};

// Buffer to store 10 acceleration readings per GPS timestamp
vector<AccData> acc_buffer;
constexpr int ACC_SAMPLES_PER_GPS = 10; // 100Hz Acc and 10Hz GPS

// Constants for SPI setup
#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED 10000000
#define SPI_BITS_PER_WORD 8

#define ADXL355_CS_PIN 8  // Hardware connection
#define READ_BIT 0x01

// Registers
#define ADXL355_REG_DEVID     0x02
#define ADXL355_REG_POWER_CTL 0x2D
#define ADXL355_REG_RANGE     0x2C
#define ADXL355_REG_XDATA3    0x08
#define ADXL355_REG_XDATA2    0x09
#define ADXL355_REG_XDATA1    0x0A
#define ADXL355_REG_FILTER    0x28 // 0101 

constexpr float SCALE = 3.9e-6f;       // Scale factor for raw data conversion to 'g'
constexpr float GRAVITY = 9.80665f;    // Standard gravity in m/s²

#define RANGE_2G 0x01
#define ADXL355_REG_POWER_CTL_STANDBY 0x01

// GPS Data Structure
struct GPSData {
    string time;
    string latitude;
    string longitude;
};

// Setup SPI and GPIO
void setupGPIO() {
    wiringPiSetupGpio();
    pinMode(ADXL355_CS_PIN, OUTPUT);
    digitalWrite(ADXL355_CS_PIN, HIGH);
}

int setupSPI() {
    int fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0) {
        perror("SPI open failed");
        return -1;
    }

    uint8_t mode = SPI_MODE_0;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {  
        perror("SPI mode set failed");
        close(fd);
        return -1;
    }

    uint32_t speed = SPI_SPEED;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {  
        perror("SPI speed set failed");
        close(fd);
        return -1;
    }
    return fd;
}

void readRegister(int fd, uint8_t reg, uint8_t *val, int len) {
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = (reg << 1) | READ_BIT;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len + 1,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS_PER_WORD,
        .cs_change = 0
    };

    digitalWrite(ADXL355_CS_PIN, LOW);
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
        perror("SPI read failed");
    digitalWrite(ADXL355_CS_PIN, HIGH);

    memcpy(val, rx + 1, len);
}

void writeRegister(int fd, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (reg << 1) & ~READ_BIT, val };
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = 0,
        .len = 2,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS_PER_WORD,
        .cs_change = 0
    };

    digitalWrite(ADXL355_CS_PIN, LOW);
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
        perror("SPI write failed");
    digitalWrite(ADXL355_CS_PIN, HIGH);
}

void setMeasureMode(int fd) {
    uint8_t current;
    readRegister(fd, ADXL355_REG_POWER_CTL, &current, 1);
    writeRegister(fd, ADXL355_REG_POWER_CTL, current & ~ADXL355_REG_POWER_CTL_STANDBY);
}

void setRange(int fd, uint8_t range) {
    uint8_t current;
    readRegister(fd, ADXL355_REG_RANGE, &current, 1);
    current = (current & 0xFC) | range;  
    writeRegister(fd, ADXL355_REG_RANGE, current);
}

void setFilter(int fd) {
    uint8_t current;
    readRegister(fd, ADXL355_REG_FILTER, &current, 1);
    current = (current & 0xF0) | 0x05;  
    writeRegister(fd, ADXL355_REG_FILTER, current);
}

int32_t mergeData(uint8_t a, uint8_t b, uint8_t c) {
    int32_t raw_data;
    ((uint8_t*)&raw_data)[1] = c;
    ((uint8_t*)&raw_data)[2] = b;
    ((uint8_t*)&raw_data)[3] = a;

    raw_data /= 4096;

    return raw_data;
}

// GPS Reading Function (10Hz)
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
                    // Split by commas
                    size_t comma1 = line.find(",");
                    size_t comma2 = line.find(",", comma1 + 1);
                    size_t comma3 = line.find(",", comma2 + 1);
                    size_t comma4 = line.find(",", comma3 + 1);
                    size_t comma5 = line.find(",", comma4 + 1);
                    size_t comma6 = line.find(",", comma5 + 1);
                
                    if (comma6 != string::npos) {
 
                        latest_gps_time = line.substr(comma1 + 1, comma2 - comma1 - 1); 
                        string latitude = line.substr(comma3 + 1, comma4 - comma3 - 1); 
                        string longitude = line.substr(comma5 + 1, comma6 - comma5 - 1);
                        latest_latitude = latitude;
                        latest_longitude = longitude;
                       // cout << "GPS Time: " << latest_gps_time << ", Latitude: " << latest_latitude << ", Longitude: " << latest_longitude << endl;
                    }
                }
                
                    gps_cv.notify_one();
                }
            }
            this_thread::sleep_for(chrono::milliseconds(10));
        }
        
    }

// ADXL355 Reading Function (100Hz)
void readADXL355(int fd) {
    while (running) {
        uint8_t data[9];
        readRegister(fd, ADXL355_REG_XDATA3, data, 9); // Read accelerometer data

        int32_t x = mergeData(data[0], data[1], data[2]);
        int32_t y = mergeData(data[3], data[4], data[5]);
        int32_t z = mergeData(data[6], data[7], data[8]);

        AccData new_acc_data;
        new_acc_data.acc_x = x * SCALE * GRAVITY;
        new_acc_data.acc_y = y * SCALE * GRAVITY;
        new_acc_data.acc_z = z * SCALE * GRAVITY;

        auto now = system_clock::now();
        auto now_time_t = system_clock::to_time_t(now);
        auto now_time = *std::localtime(&now_time_t);
    
        // Get precise time fractions
        auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
        auto now_us = duration_cast<microseconds>(now.time_since_epoch()) % 1000000;
    
        // Format time string
        stringstream time_stream;
        time_stream << put_time(&now_time, "%H:%M:%S") << "."
                    << setw(3) << setfill('0') << now_ms.count()
                    << setw(3) << setfill('0') << (now_us.count() % 1000);

        new_acc_data.time = time_stream.str();  // Store time as string in the data structure

        // Output acceleration data with time
        cout << new_acc_data.acc_x << "," << new_acc_data.acc_y << "," << new_acc_data.acc_z << "," << new_acc_data.time << endl;
        
        {
            lock_guard<mutex> lock(data_lock);
            acc_buffer.push_back(new_acc_data);
        }

        this_thread::sleep_for(chrono::milliseconds(10)); // Run at 100Hz
    }
}

string generateFilename() {
    auto now = chrono::system_clock::now();
    time_t now_time_t = chrono::system_clock::to_time_t(now);
    struct tm *time_info = localtime(&now_time_t);

    stringstream filename;
    filename << "sensor_data_"
             << put_time(time_info, "%Y-%m-%d_%H-%M-%S") << ".csv";
    
    return filename.str();
}

void writeToCSV() {
    string filename = generateFilename();
    // cout << "Writing to file: " << filename << endl;

    ofstream file(filename, ios::app);
    if (!file.is_open()) {
        cerr << "Error: Failed to open file: " << filename << endl;
        return;
    }

    // Write header if file is empty
    if (file.tellp() == 0) {
        file << "UTC_Time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, Latitude, Longitude" << endl;
    }

    while (running) {
        string gps_time, latitude, longitude;

        {
            unique_lock<mutex> lock_gps(gps_lock);
            gps_cv.wait(lock_gps, [] { return !latest_gps_time.empty(); });

            gps_time = latest_gps_time;
            latitude = latest_latitude;
            longitude = latest_longitude;
        }  // Lock is released here

        vector<AccData> buffer_copy;

        {
            lock_guard<mutex> data_lock_guard(data_lock);
            buffer_copy = acc_buffer;  // Copy the buffer
            acc_buffer.clear();        // Clear the original buffer
        }

        // Write data to the CSV file
        for (const auto &acc : buffer_copy) {
            file << gps_time << "," 
                 << acc.acc_x << "," << acc.acc_y << "," << acc.acc_z << ","  
                 << "nan,nan,nan,"  // Assuming no gyro data is available
                 << latitude << "," << longitude << "," << acc.time << endl;
            file.flush();  // Flush the buffer
        }

        // cout << "Wrote " << buffer_copy.size() << " records to file." << endl;
    }

    file.close();
}

int main() {
    int serial_fd = serialOpen("/dev/ttyAMA0", 115200);
    if (serial_fd == -1) {
        cerr << "Failed to open GPS serial port!" << endl;
        return 1;
    }

    setupGPIO();
    int fd = setupSPI();
    if (fd < 0) return 1;

    uint8_t devId;
    readRegister(fd, ADXL355_REG_DEVID, &devId, 1);
    cout << "Device ID: 0x" << hex << (int)devId << endl;

    if (devId != 0xED) {
        cerr << "not match!" << endl;
        close(fd);
        return 1;
    }

    setRange(fd, RANGE_2G);
    setFilter(fd);
    setMeasureMode(fd);

    thread gps_thread(readGPS, serial_fd);
    thread adxl_thread(readADXL355, fd);
    thread csv_thread(writeToCSV);

    gps_thread.join();
    adxl_thread.join();
    csv_thread.join();

    return 0;
}
