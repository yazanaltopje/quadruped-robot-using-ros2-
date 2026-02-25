#ifndef ARDUINO_DRIVER_HPP
#define ARDUINO_DRIVER_HPP

#include <string>
#include <vector>

namespace qu_hardware
{

class ArduinoDriver
{
public:
    
    ArduinoDriver(const std::string& port, int baud_rate = 115200);
    
   
    ~ArduinoDriver();
    
   
    bool connect();
    
   
    void disconnect();
    
 
    bool isConnected() const;
    
   
    bool setServoAngle(int servo_id, double angle_deg);
    
   
    bool getServoAngle(int servo_id, double& angle_deg);

private:
    std::string port_;          
    int baud_rate_;             
    int serial_fd_;              
    bool connected_;            
    
  
    bool sendCommand(const std::string& cmd);
    std::string readLine();
};

}

#endif 
