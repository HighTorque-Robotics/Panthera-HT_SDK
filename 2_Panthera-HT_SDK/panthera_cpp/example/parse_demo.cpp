#include <iostream>
#include "parse_robot_params.hpp"
int main(int argc, char* argv[]) {
    std::string filePath = "../robot_param/1dof_STM32H730_model_test_Orin_params.yaml";
    RobotParams params = parseRobotParams(filePath);

    std::cout << "Robot Name: " << params.robot_name << std::endl;
    std::cout << "Serial Type: " << params.Serial_Type << std::endl;
    std::cout << "Serial Baudrate: " << params.Seial_baudrate << std::endl;
    std::cout << "CAN Board Num: " << params.CANboard_num << std::endl;

    for (const auto& board : params.CANboards) {
        std::cout << "CAN Board: " << board.first << std::endl;
        for (const auto& port : board.second.CANports) {
            std::cout << "  CAN Port: " << port.first << std::endl;
            
            for (const auto& motor : port.second.motors) {
                std::cout << "    Motor: " << motor.first << std::endl;
                std::cout << "      Type: " << motor.second.type << std::endl;
                std::cout << "      ID: " << motor.second.id << std::endl;
                std::cout << "      Name: " << motor.second.name << std::endl;
                std::cout << "      Num: " << motor.second.num << std::endl;
                std::cout << "      Pos Limit Enable: " << motor.second.pos_limit_enable << std::endl;
                std::cout << "      Pos Upper: " << motor.second.pos_upper << std::endl;
                std::cout << "      Pos Lower: " << motor.second.pos_lower << std::endl;
                std::cout << "      Tor Limit Enable: " << motor.second.tor_limit_enable << std::endl;
                std::cout << "      Tor Upper: " << motor.second.tor_upper << std::endl;
                std::cout << "      Tor Lower: " << motor.second.tor_lower << std::endl;
            }
        }
    }

    return 0;
}