#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <wiringPi.h>
#include <wiringSerial.h>

std::atomic<bool> running(true); 

// Function to read GPS data in a separate thread
void readGPS() {
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
                    return line;
                }
            }
        }
        //this_thread::sleep_for(milliseconds(10));  
    }
}
    
void readSensor() {
    while (running) {
       
        std::cout << "Reading sensor data...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    }
}

int main() {
    wiringPiSetup(); 

    std::thread gpsThread(readGPS);  
    std::thread sensorThread(readSensor); 

    std::cout << "Press Enter to stop...\n";
    std::cin.get(); 

    running = false; 

    gpsThread.join();    
    sensorThread.join();

    std::cout << "Program terminated.\n";
    return 0;
}
