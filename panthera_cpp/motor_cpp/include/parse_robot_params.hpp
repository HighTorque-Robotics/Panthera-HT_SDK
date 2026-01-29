#ifndef PARSE_ROBOT_PARAMS_HPP
#define PARSE_ROBOT_PARAMS_HPP

#include <yaml-cpp/yaml.h>


struct MotorNameComparator 
{
    bool operator()(const std::string& a, const std::string& b) const;
    
private:
    int extractNumber(const std::string& str) const;
};



struct MotorParams 
{
    std::string type;
    int id;
    std::string name;
    int num;
    bool pos_limit_enable;
    double pos_upper;
    double pos_lower;
    bool tor_limit_enable;
    double tor_upper;
    double tor_lower;
};

struct CANPortParams 
{
    int serial_id;
    int motor_num;    
    std::map<std::string, MotorParams, MotorNameComparator> motors;
};

struct CANBoardParams {
    int CANport_num;
    std::map<std::string, CANPortParams, MotorNameComparator> CANports;
};

struct RobotParams 
{
    int motor_timeout_ms;
    std::string robot_name;
    std::string Serial_Type;
    int Seial_baudrate;
    int CANboard_num;
    bool board_special_flag;
    bool canport_error_output_flag;
    bool exit_motor_brake_flag;
    std::map<std::string, CANBoardParams, MotorNameComparator> CANboards;
};

RobotParams parseRobotParams(const std::string& filePath);

#endif
