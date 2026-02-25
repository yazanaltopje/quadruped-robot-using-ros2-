#include "qu_hardware/arduino_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <cstring>

namespace qu_hardware
{

ArduinoDriver::ArduinoDriver(const std::string& port, int baudrate)
    : port_(port), baud_rate_(baudrate), serial_fd_(-1), connected_(false) {
    std::cout << "[ArduinoDriver] Created for port: " << port << std::endl;
}

ArduinoDriver::~ArduinoDriver() {
    if (connected_) {
        disconnect();
    }
}

bool ArduinoDriver::connect() {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    
    if (serial_fd_ == -1) {
        std::cerr << "[ArduinoDriver] ERROR: Cannot open port " << port_ << std::endl;
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "[ArduinoDriver] ERROR: tcgetattr failed" << std::endl;
        close(serial_fd_);
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[ArduinoDriver] ERROR: tcsetattr failed" << std::endl;
        close(serial_fd_);
        return false;
    }

    usleep(2000000);
    connected_ = true;
    
    std::cout << "[ArduinoDriver] Connected to " << port_ << std::endl;
    return true;
}

void ArduinoDriver::disconnect() {
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    connected_ = false;
    std::cout << "[ArduinoDriver] Disconnected" << std::endl;
}

bool ArduinoDriver::isConnected() const {
    return connected_;
}

bool ArduinoDriver::setServoAngle(int servo_id, double angle_deg) {
    if (!connected_) {
        return false;
    }

    std::ostringstream cmd;
    cmd << servo_id << ":" << static_cast<int>(angle_deg) << "\n";
    std::string command = cmd.str();

    ssize_t bytes_written = write(serial_fd_, command.c_str(), command.length());
    
    if (bytes_written < 0) {
        std::cerr << "[ArduinoDriver] Write failed" << std::endl;
        return false;
    }

    std::cout << "[Servo " << servo_id << "] → " << angle_deg << "°" << std::endl;
    
    return true;
}

bool ArduinoDriver::getServoAngle(int servo_id, double& angle_deg) {
    return false;
}

bool ArduinoDriver::sendCommand(const std::string& cmd) {
    return false;
}

std::string ArduinoDriver::readLine() {
    return "";
}

} // namespace qu_hardware