#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>
#include "canboard.hpp"
#include <thread>
#include <initializer_list>
#include <fstream>
#include <libserialport.h>
#include <dirent.h>
#include <algorithm>
#include <lcm/lcm-cpp.hpp>

namespace hightorque_robot
{
    class robot
    {
    private:
        std::string robot_name, Serial_Type;
        int CANboard_num, Seial_baudrate;
        std::vector<canboard> CANboards;
        std::vector<std::string> str;
        std::string SDK_version2 = "4.6.0"; // SDK版本
        std::condition_variable error_check_cv;
        std::mutex error_check_mutex;
        bool error_check_flag = false;
        std::thread error_check_thread_;
        std::thread pub_thread_;
        fun_version fun_v = fun_v1;
        uint16_t slave_v = COMBINE_VERSION(3, 0, 0);
        std::shared_ptr<lcm::LCM> lcm_ptr;
        bool lcm_en;
        bool canport_error_output_flag = false;
        bool board_special_flag = false;
    public:
        std::vector<serial_driver *> ser;
        std::vector<motor *> Motors;
        std::vector<canport *> CANPorts;
        std::vector<std::thread> ser_recv_threads;
        int motor_position_limit_flag = 0;
        int motor_torque_limit_flag = 0;

        RobotParams robot_params;

        int motor_timeout_ms = 0;

        robot();
        robot(const std::string& config_path);
        ~robot();

    private:
        void init_robot(const std::string& config_path);
        int serial_pid_vid(const char *name, int *pid, int *vid);
        int serial_pid_vid(const char *name);
        std::vector<std::string> list_serial_ports(const std::string& full_prefix);
        void init_ser();        
        void check_error();
        int check_serial_dev_exist(int);        
        void set_port_motor_num();
    public:

        void publishJointStates();
        void detect_motor_limit();
        void motor_send_cmd();       
        void send_get_motor_state_cmd();
        void send_get_motor_version_cmd();
        void check_motor_connection_position();
        void check_motor_connection_version();
        void set_brake();
        void set_stop();
        void set_reset();
        void set_reset_zero();
        void set_reset_zero(std::initializer_list<int> motors);
        void set_motor_runzero();
        void set_timeout(int16_t t_ms);
        void set_timeout(uint8_t portx, int16_t t_ms);
        void motor_version_detection();
        void set_data_reset();
        void canboard_bootloader();
        void lcm_enable();
        void canboard_fdcan_reset();
        void get_motor_tqe_adjs_flag();
        void send_get_tqe_adjust_flag_cmd();
        void check_tqe_adjust_flag();
    };
}
#endif
