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
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <condition_variable>
#include <cstring>

#define SERIAL_PORT "/dev/ttyAMA0"
#define BAUDRATE B115200

using namespace std;
using namespace std::chrono;

// Global Variables
mutex gps_lock;
condition_variable gps_cv;
int serial_fd;

struct GPSData {
    string time;
    string latitude = "";
    string longitude = "";
};

GPSData latest_gps{ "nan", "nan", "nan" };
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
                cout << line << endl;  // 

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
    }
}

void main_thread() {
    string file_name = "gps_log_" + to_string(time(nullptr)) + ".csv";
    ofstream file(file_name, ios::app); // Append mode
    if (!file.is_open()) {
        cerr << "Error opening file: " << file_name << endl;
        return;
    }

    file << "UTC_Time,Latitude,Longitude,Timestamp\n";
    file.flush();

    cout << "Logging GPS data to: " << file_name << endl;

    while (true) {
        auto start_time = steady_clock::now();

        GPSData gps_data;
        {
            unique_lock<mutex> lock(gps_lock);
            if (gps_cv.wait_for(lock, milliseconds(50), [] { return !latest_gps.time.empty(); })) {
                gps_data = latest_gps;
                cout << "GPS Data Updated: " << gps_data.time << endl;
            } else {
                cout << "No GPS data available." << endl;
                continue; // Skip writing if no data
            }
        }

        auto now = system_clock::now();
        time_t now_c = system_clock::to_time_t(now);
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        file << gps_data.time << ","
             << gps_data.latitude << ","
             << gps_data.longitude << ","
             << put_time(localtime(&now_c), "%Y-%m-%d %H:%M:%S")
             << '.' << setfill('0') << setw(3) << ms.count()
             << "\n";

        file.flush();  // Force writing to the file

        cout << "GPS Data: " << gps_data.time << " | "
             << gps_data.latitude << ", " << gps_data.longitude << endl;

        auto end_time = steady_clock::now();
        auto elapsed_time = duration_cast<milliseconds>(end_time - start_time).count();
        auto sleep_time = 100 - elapsed_time;
        if (sleep_time > 0) {
            this_thread::sleep_for(milliseconds(sleep_time));
        }
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
