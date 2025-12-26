#include <iostream>
#include "parse_robot_params.hpp"


bool MotorNameComparator::operator()(const std::string& a, const std::string& b) const 
{
    int numA = extractNumber(a);
    int numB = extractNumber(b);
    return numA < numB;
}

int MotorNameComparator::extractNumber(const std::string& str) const 
{
    std::string numberStr;
    
    for (char ch : str) 
    {
        if (std::isdigit(ch)) 
        {
            numberStr += ch;
        } 
        else if (!numberStr.empty()) 
        {
            break;
        }
    }
    
    if (numberStr.empty()) 
    {
        throw std::invalid_argument("No numeric value found in string: " + str);
    }
    
    return std::stoi(numberStr);
}



static void printParams(const RobotParams &params)
{
    for (const auto &boardEntry : params.CANboards)
    {
        std::cout << "CAN Board Name: " << boardEntry.first << std::endl;
        const CANBoardParams &board = boardEntry.second;
        std::cout << "  CAN Port Num: " << board.CANport_num << std::endl;
        for (const auto &portEntry : board.CANports)
        {
            std::cout << "  CAN Port Name: " << portEntry.first << std::endl;
            const CANPortParams &port = portEntry.second;
            std::cout << "    Serial ID: " << port.serial_id << std::endl;
            std::cout << "    Motor Num: " << port.motor_num << std::endl;
            for (const auto &motorEntry : port.motors)
            {
                const MotorParams &motor = motorEntry.second;
                std::cout << "      Type: " << motor.type << std::endl;
                std::cout << "      ID: " << motor.id << std::endl;
                std::cout << "      Name: " << motor.name << std::endl;
                std::cout << "      Num: " << motor.num << std::endl;
                std::cout << "      Position Limit Enabled: " << (motor.pos_limit_enable ? "True" : "False") << std::endl;
                std::cout << "      Position Upper Limit: " << motor.pos_upper << std::endl;
                std::cout << "      Position Lower Limit: " << motor.pos_lower << std::endl;
                std::cout << "      Torque Limit Enabled: " << (motor.tor_limit_enable ? "True" : "False") << std::endl;
                std::cout << "      Torque Upper Limit: " << motor.tor_upper << std::endl;
                std::cout << "      Torque Lower Limit: " << motor.tor_lower << std::endl;
            }
        }
    }
}

template <typename T>
void readConfigParam(const YAML::Node &node, const std::string &key, T &value)
{
    if (node[key])
    {
        try
        {
            value = node[key].as<T>();
            // std::cout << key << ": " << value << std::endl;
        }
        catch (const YAML::BadConversion &e)
        {
            std::cerr << "\033[1;31m" << "Error: Failed to convert '" << key << "' to the required type: " << e.what() << "\033[0m" << std::endl;
            exit(-1);
        }
    }
    else
    {
        std::cerr << "\033[1;31m" << "Error: '" << key << "' is missing in configuration file." << "\033[0m" << std::endl;
        exit(-1);
    }
}


template <typename T>
void readConfigParam(const YAML::Node &node, const std::string &key, T &value, T default_value)
{
    if (node[key])
    {
        try
        {
            value = node[key].as<T>();
            // std::cout << key << ": " << value << std::endl;
        }
        catch (const YAML::BadConversion &e)
        {
            std::cerr << "\033[1;31m" << "Error: Failed to convert '" << key << "' to the required type: " << e.what() << "\033[0m" << std::endl;
            exit(-1);
        }
    }
    else
    {
        value = default_value;
    }
}


RobotParams parseRobotParams(const std::string &filePath)
{
    RobotParams params;
    YAML::Node config = YAML::LoadFile(filePath);

    if (config["robot"])
    {
        YAML::Node robotNode = config["robot"];
        readConfigParam(robotNode, "robot_name", params.robot_name);
        readConfigParam(robotNode, "Serial_Type", params.Serial_Type);
        readConfigParam(robotNode, "Seial_baudrate", params.Seial_baudrate);
        readConfigParam(robotNode, "motor_timeout_ms", params.motor_timeout_ms);
        readConfigParam(robotNode, "CANboard_num", params.CANboard_num);
        readConfigParam(robotNode, "canport_error_output_flag", params.canport_error_output_flag, false);
        readConfigParam(robotNode, "board_special_flag", params.board_special_flag, false);
        readConfigParam(robotNode, "exit_motor_brake_flag", params.exit_motor_brake_flag, false);

        if (robotNode["CANboard"])
        {
            YAML::Node CANboardNode = robotNode["CANboard"];
            int board_num = 0;
            for (YAML::const_iterator it = CANboardNode.begin(); it != CANboardNode.end() && board_num < params.CANboard_num; ++it, ++board_num)
            {
                std::string boardName = it->first.as<std::string>();
                YAML::Node boardNode = it->second;
                CANBoardParams board;
                readConfigParam(boardNode, "CANport_num", board.CANport_num);

                if (boardNode["CANport"])
                {
                    YAML::Node CANportNode = boardNode["CANport"];
                    int port_num = 0;
                    for (YAML::const_iterator portIt = CANportNode.begin(); portIt != CANportNode.end() && port_num < board.CANport_num; ++portIt, ++port_num)
                    {
                        std::string portName = portIt->first.as<std::string>();
                        YAML::Node portNode = portIt->second;
                        CANPortParams port;
                        readConfigParam(portNode, "serial_id", port.serial_id);
                        readConfigParam(portNode, "motor_num", port.motor_num);

                        if (portNode["motor"])
                        {
                            YAML::Node motorNode = portNode["motor"];
                            int motor_num = 0;
                            for (YAML::const_iterator motorIt = motorNode.begin(); motorIt != motorNode.end() && motor_num < port.motor_num; ++motorIt, ++motor_num)
                            {
                                std::string motorName = motorIt->first.as<std::string>();
                                YAML::Node motorData = motorIt->second;
                                MotorParams motor;
                                readConfigParam(motorData, "type", motor.type);
                                readConfigParam(motorData, "id", motor.id);
                                readConfigParam(motorData, "name", motor.name);
                                readConfigParam(motorData, "num", motor.num);
                                readConfigParam(motorData, "pos_limit_enable", motor.pos_limit_enable);
                                readConfigParam(motorData, "pos_upper", motor.pos_upper);
                                readConfigParam(motorData, "pos_lower", motor.pos_lower);
                                readConfigParam(motorData, "tor_limit_enable", motor.tor_limit_enable);
                                readConfigParam(motorData, "tor_upper", motor.tor_upper);
                                readConfigParam(motorData, "tor_lower", motor.tor_lower);
                                port.motors[motorName] = motor;
                            }
                        }
                        board.CANports[portName] = port;
                    }
                }
                params.CANboards[boardName] = board;
            }
        }
    }

    // printParams(params);

    return params;
}