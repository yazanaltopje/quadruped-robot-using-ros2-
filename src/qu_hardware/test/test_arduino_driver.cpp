#include "qu_hardware/arduino_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Arduino Driver Test Program" << std::endl;
    std::cout << "========================================" << std::endl;
    
    
    std::string port = "/dev/ttyACM0";  
    qu_hardware::ArduinoDriver driver(port);
    
    
    std::cout << "\nConnecting to Arduino..." << std::endl;
    if (!driver.connect()) {
        std::cerr << "Failed to connect! Check:" << std::endl;
        std::cerr << "  1. Arduino is connected" << std::endl;
        std::cerr << "  2. Port is correct: " << port << std::endl;
        std::cerr << "  3. You have permissions (try: sudo chmod 666 " << port << ")" << std::endl;
        return 1;
    }
    
    std::cout << "Connected successfully!" << std::endl;
    
   
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    
    std::cout << "\n--- Testing Servo Movement ---" << std::endl;
    
    std::vector<double> test_angles = {0, 45, 90, 135, 180, 90};
    
    for (double angle : test_angles) {
        std::cout << "\nMoving servo to " << angle << "°..." << std::endl;
        
        if (driver.setServoAngle(0, angle)) {
            std::cout << "Command sent successfully" << std::endl;
        } else {
            std::cerr << "Failed to send command!" << std::endl;
        }
        
       
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        
        double current_angle;
        if (driver.getServoAngle(0, current_angle)) {
            std::cout << "Current position: " << current_angle << "°" << std::endl;
        }
    }
    
    std::cout << "\n--- Test Complete! ---" << std::endl;
    std::cout << "Disconnecting..." << std::endl;
    
    return 0;
}