#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyAMA0"  // Change this as needed

int main() {
    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port!" << std::endl;
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting serial attributes!" << std::endl;
        close(serial_fd);
        return 1;
    }

    // Configure serial port settings
    cfsetispeed(&tty, B115200);  
    cfsetospeed(&tty, B115200);  

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 
    tty.c_cflag |= CREAD | CLOCAL;             
    tty.c_lflag &= ~ICANON;                    
    tty.c_lflag &= ~(ECHO | ECHOE | ISIG);     
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);    
    tty.c_oflag &= ~OPOST;                     

    tty.c_cc[VMIN] = 1;  
    tty.c_cc[VTIME] = 10; 

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes!" << std::endl;
        close(serial_fd);
        return 1;
    }

    // **Flush the serial port before reading**
    tcflush(serial_fd, TCIFLUSH);

    std::cout << "Reading from serial port..." << std::endl;

    char buffer[256];
    while (true) {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(serial_fd, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0) {
            std::cout << "Received: " << buffer << std::endl;
        }
    }

    close(serial_fd);
    return 0;
}
