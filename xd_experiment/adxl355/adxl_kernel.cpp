#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include <thread>
#include <chrono>
#include <cstring>

using namespace std;

// SPI setup
#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED 10000000 // 降低至500kHz测试
#define SPI_BITS_PER_WORD 8

#define ADXL355_CS_PIN 8  // 确认硬件连接
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

// Range settings
#define RANGE_2G 0x01
#define ADXL355_REG_POWER_CTL_STANDBY 0x01

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
    cout << "0x" << hex << (int)current << " " << endl;
    writeRegister(fd, ADXL355_REG_POWER_CTL, current & ~ADXL355_REG_POWER_CTL_STANDBY);
}

void setRange(int fd, uint8_t range) {
    uint8_t current;
    readRegister(fd, ADXL355_REG_RANGE, &current, 1);
    cout << "Range read: 0x" << hex << (int)current << " ";
    current = (current & 0xFC) | range;  
    cout << "Range set: 0x" << hex << (int)current << " " << endl;
    writeRegister(fd, ADXL355_REG_RANGE, current);
}

void setFilter(int fd) {
    uint8_t current;
    readRegister(fd, ADXL355_REG_FILTER, &current, 1);
    cout << "Range read: 0x" << hex << (int)current << " ";
    current = (current & 0xF0) | 0x05;  
    cout << "Range set: 0x" << hex << (int)current << " " << endl;
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

// Function to read acceleration data and update values directly
void readAccelData(int fd,int32_t *raw_x, int32_t *raw_y, int32_t *raw_z) {
    uint8_t data[9];
    readRegister(fd, ADXL355_REG_XDATA3, data, 9);

    // Dereference pointers and assign the merged data
    *raw_x = mergeData(data[0], data[1], data[2]);
    *raw_y = mergeData(data[3], data[4], data[5]);
    *raw_z = mergeData(data[6], data[7], data[8]);

    // cout << "X: " << *raw_x << " Y: " << *raw_y << " Z: " << *raw_z << endl;
}

int main() {
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
    int32_t x, y, z;
    float acc_x,acc_y,acc_z;
    try {
        while (true) {
            readAccelData(fd,&x, &y, &z);
            acc_x = x * SCALE * GRAVITY;
            acc_y = y *  SCALE * GRAVITY;
            acc_z = z * SCALE * GRAVITY;

            // writeToFile(chrono::utc_clock::now(), acc_x,acc_y,acc_z);

            cout << acc_x << "," <<  acc_y <<  ","<< acc_z << endl;
            this_thread::sleep_for(chrono::milliseconds(10));
        }
    } catch (...) {
        close(fd);
        return 1;
    }

    close(fd);
    return 0;
}