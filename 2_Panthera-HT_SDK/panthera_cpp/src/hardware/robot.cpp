#include "robot.hpp"
#include "parse_robot_params.hpp"
#include <unistd.h>
#include "motor_msg/motor_msg.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>

namespace hightorque_robot
{
    robot::robot()
    {
        init_robot("../robot_param/robot_config.yaml");
    }

    robot::robot(const std::string& config_path)
    {
        init_robot(config_path);
    }

    void robot::init_robot(const std::string& config_path)
    {
        auto config = YAML::LoadFile(config_path);
        std::cout << "robot_config: " << config["robot"]["name"].as<std::string>() << std::endl;
        auto param_file = config_path;
        if (config["robot"]["param_file"])
        {
            param_file = config["robot"]["param_file"].as<std::string>();
        }
        
        // 如果param_file是相对路径，则相对于config_path的目录
        if (param_file[0] != '/') {
            size_t last_slash = config_path.find_last_of('/');
            if (last_slash != std::string::npos) {
                std::string config_dir = config_path.substr(0, last_slash + 1);
                param_file = config_dir + param_file;
            }
        }
        
        robot_params  = parseRobotParams(param_file);
        Seial_baudrate = robot_params.Seial_baudrate;
        robot_name = robot_params.robot_name;
        motor_timeout_ms = robot_params.motor_timeout_ms;
        CANboard_num = robot_params.CANboard_num;
        Serial_Type = robot_params.Serial_Type;
        canport_error_output_flag = robot_params.canport_error_output_flag;
        board_special_flag = robot_params.board_special_flag;

        if (motor_timeout_ms < 0 || motor_timeout_ms > 32760)
        {
            ROS_ERROR("The value of motor_timeout_ms is out of the valid range [0, 32760]");
            exit(-1);
        }
        
        std::cout << "\033[1;32mGot params SDK_version: v" << SDK_version2 << "\033[0m" << std::endl;
        std::cout << "\033[1;32mThe robot name is " << robot_name << "\033[0m" << std::endl;
        std::cout << "\033[1;32mThe robot has " << CANboard_num << " CANboards\033[0m" << std::endl;
        std::cout << "\033[1;32mThe Serial type is " << Serial_Type << "\033[0m" << std::endl;

        init_ser();
        error_check_flag = true;
        error_check_thread_ = std::thread(&robot::check_error, this);
        auto it = robot_params.CANboards.begin();
        for (size_t i = 1; i <= CANboard_num; i++, it++)
        {
            CANboards.push_back(canboard(i, &ser, it->second, canport_error_output_flag));
        }

        for (canboard &cb : CANboards)
        {
            cb.push_CANport(&CANPorts);
        }
        for (canport *cp : CANPorts)
        {
            cp->puch_motor(&Motors);
        }
        set_port_motor_num(); // 设置通道上挂载的电机数，并获取主控板固件版本号
        if (slave_v >= COMBINE_VERSION(4, 1, 0))
        {
            canboard_fdcan_reset();
        }

        if (slave_v < COMBINE_VERSION(4, 0, 0))  // 检测电机连接是否正常
        {
            fun_v = fun_v1;
            check_motor_connection_position();   
        }
        else
        {
            fun_v = fun_v2;
            check_motor_connection_version();
        }

        if (motor_timeout_ms != 0)
        {
            set_timeout(motor_timeout_ms);
        }
        
        send_get_motor_state_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        this->lcm_en = false;

        std::cout << "\033[1;32mThe robot has " << Motors.size() << " motors\033[0m" << std::endl;
        std::cout << "robot init" << std::endl;
    }


    robot::~robot()
    {
        if (robot_params.exit_motor_brake_flag)
        {
            set_brake();
            set_brake();
            set_brake();
            printf("motor brake\n");
        }
        else
        {
            set_stop();
            set_stop();
            set_reset();
            printf("motor stop\n");
        }


        for (serial_driver *s : ser)
        {
            if (s) 
            {
                s->set_run_flag(false);
                s->close();
            }
        }

        for (auto &thread : ser_recv_threads)
        {
            if (thread.joinable())
                thread.join();
        }

        this->lcm_en = false;
        if(pub_thread_.joinable())
        {
            pub_thread_.join(); 
        }

        error_check_flag = false;
        if(error_check_thread_.joinable())
        {
            error_check_thread_.join(); 
        }
    }

    void robot::lcm_enable()
    {
        if(this->lcm_en == false)
        {
            this->lcm_en = true;
            lcm_ptr = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=0");
            if (!lcm_ptr->good())
            {
                std::cerr << "\033[1;31m" << "LCM init error" << "\033[0m" << std::endl;
            }
            else
            {
                std::cout << std::endl <<  "LCM init success" << std::endl;
                pub_thread_ = std::thread(&robot::publishJointStates, this);
            }
        }
    }

    void robot::publishJointStates()
    {
        while(this->lcm_en)
        {
            motor_msg::motor_msg msg;
            for(int i = 0; i < 40; i++)
            {
                msg.motor_status[i] = 0;
            }
            msg.can1_num = 0;
            msg.can2_num = 0;
            msg.can3_num = 0;
            msg.can4_num = 0;
            if(CANPorts.size() > 0)
            {
                msg.can1_num = CANPorts[0]->get_motor_num();
            }
            if(CANPorts.size() > 1)
            {
                msg.can2_num = CANPorts[1]->get_motor_num();
            }
            if(CANPorts.size() > 2)
            {
                msg.can3_num = CANPorts[2]->get_motor_num();
            }
            if(CANPorts.size() > 3)
            {
                msg.can4_num = CANPorts[3]->get_motor_num();
            }
            int cnt = 0;
            for (motor *m : Motors)
            {    
                motor_back_t* data_ptr=m->get_current_motor_state();
                auto now = std::chrono::system_clock::now();
                auto now_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;
                if(now_time - data_ptr->time < 0.1)
                {   
                    msg.motor_status[cnt] = data_ptr->position;
                }
                else
                {
                    msg.motor_status[cnt] = -999.0f;
                }
                cnt ++;
            }
            // Publish the joint state message
            lcm_ptr->publish("motor_msg", &msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void robot::detect_motor_limit()
    {
        // 电机正常运行时检测是否超过限位，停机之后不检测
        if(!motor_position_limit_flag && !motor_torque_limit_flag)
        {
            for (motor *m : Motors)
            {
                if(m->pos_limit_flag)
                {                    
                    std::cerr << "\033[1;31m" << "robot pos limit, motor stop." << "\033[0m" << std::endl;
                    set_stop();
                    motor_position_limit_flag = m->pos_limit_flag;
                    break;
                }

                if(m->tor_limit_flag)
                {
                    std::cerr << "\033[1;31m" << "robot torque limit, motor stop." << "\033[0m" << std::endl;
                    set_stop();
                    motor_torque_limit_flag = m->tor_limit_flag;
                    break;
                }
            }            
        }
    }

    void robot::motor_send_cmd()
    {
        if(!motor_position_limit_flag && !motor_torque_limit_flag)
        {
            for (canboard &cb : CANboards)
            {
                cb.motor_send_cmd();
            }
        }
        
    }


    int robot::serial_pid_vid(const char *name, int *pid, int *vid)
    {
        int r = 0;
        struct sp_port *port;
        try
        {
            /* code */
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, vid, pid) != SP_OK) 
            {
                r = 1;
            } 
            std::cout << "Port: " << name << ", PID: 0x" << std::hex << *pid << ", VID: 0x" << *vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);
        }
        catch(const std::exception& e)
        {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << '\n';
            sp_close(port);
            sp_free_port(port);
        }
        return r;
    }


    int robot::serial_pid_vid(const char *name)
    {
        int pid, vid;
        int r = 0;
        struct sp_port *port;
        try
        {
            sp_get_port_by_name(name, &port);
            if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
            {
                r = -1;
            } 
            else 
            {
                if (pid == 0xFFFF)
                {
                    if (board_special_flag)
                    {
                        if (vid == 0xFAE1)
                        {
                            r = 1;
                        }
                        else
                        {
                            r = -3;
                        }
                    }
                    else
                    {
                        switch (vid)
                        {
                        case (0xCAF1):
                        case (0xCAE1):
                            r = 1;
                            break;
                        default:
                            r = -3;
                            break;
                        }
                    }
                }
                else
                {
                    r = -1;
                }
            }
            // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

            // 关闭端口
            sp_free_port(port);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            r = -3;
            sp_close(port);
            sp_free_port(port);
        }

        return r;
    }


    std::vector<std::string> robot::list_serial_ports(const std::string& full_prefix) 
    {
        std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
        std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
        std::vector<std::string> serial_ports;
        DIR *directory;
        struct dirent *entry;

        directory = opendir(base_path.c_str());
        if (!directory)
        {
            std::cerr << "\033[1;31m << Could not open the directory " << base_path << "\033[0m" << std::endl;
            return serial_ports; 
        }

        while ((entry = readdir(directory)) != NULL)
        {
            std::string entryName = entry->d_name;
            if (entryName.find(prefix) == 0)
            { 
                serial_ports.push_back(base_path + entryName);
            }
        }

        closedir(directory);

        std::reverse(serial_ports.begin(), serial_ports.end());

        return serial_ports;
    }


    void robot::init_ser()
    {
        ser.clear();
        ser_recv_threads.clear();
        str.clear();   
        std::vector<std::string> ports = list_serial_ports(Serial_Type);
        std::cout << "Serial Port List: " << std::endl;
        int8_t board_port_num = 99;
        for (const std::string& port : ports) 
        {   
            const int8_t r = serial_pid_vid(port.c_str());
            if (r > 0)
            {
                std::cout << "Serial Port" << str.size() << " = " << port << std::endl;
                str.push_back(port);
            }
        }

        for(auto board_params: robot_params.CANboards)
        {
            auto cp_num = board_params.second.CANport_num;
            std::vector<int> serial_id_old;
            for(auto port_params: board_params.second.CANports)
            {
                int serial_id = port_params.second.serial_id;

                serial_id_old.push_back(serial_id);

                serial_driver *s = new serial_driver(&str[serial_id - 1], Seial_baudrate, canport_error_output_flag);
                ser.push_back(s);
                ser_recv_threads.push_back(std::thread(&serial_driver::recv_1for6_42, s));
            }
        }
    }

    typedef enum{
        error_check = 0,    // 正常
        error_clear,        // 报错，清理     
        error_wait_dev,     // 报错，等待设备
        error_reconnect,    // 报错，重连
    }error_run_state_e;

    void robot::check_error(void)
    {
        std::mutex robot_mutex;
        while(error_check_flag)
        {
            static error_run_state_e last_error_run_state = error_reconnect;
            static error_run_state_e error_run_state = error_check;// 0：正常，1：报错,清理，2：重连
            switch(error_run_state)
            {
                case 0:
                {
                    bool serial_error = false;
                    for (serial_driver *s : ser)
                    {
                        if (s->is_serial_error())
                        {
                            serial_error = true;
                            break;
                        }
                    }
                    if(serial_error)
                    {
                        serial_error = false;
                        error_run_state = error_clear;
                        std::cerr << "\033[1;31mSerial error\033[0m" << std::endl;
                    }
                }
                break;
                case error_clear:
                {
                    std::lock_guard<std::mutex> lock(robot_mutex);
                    for (serial_driver *s : ser)
                    {
                        s->set_run_flag(false);
                        // s->close();
                    }
                    for (auto &_thread : ser_recv_threads)
                    {
                        if (_thread.joinable())
                        {
                            _thread.join();
                        }
                    }

                    CANboards.clear();
                    CANPorts.clear();
                    Motors.clear();
        
                    for (serial_driver *s : ser)
                    {
                        delete s;
                    }

                    ser.clear();
                    error_run_state = error_wait_dev;
                    std::cerr << "\033[1;31mclear obj and thread\033[0m" << std::endl;
                }
                break;
                case error_wait_dev:
                {
                    int exist_num = this->check_serial_dev_exist(8);
                    std::cerr << "\033[1;31mfind "  << exist_num << " device(s)\033[0m" << std::endl;
                    if (exist_num < 4)
                    {
                        std::cerr << "\033[1;31mCannot find 4 motor serial port, please check if the USB connection is normal.\033[0m" << std::endl;
                    }
                    else
                    {
                        std::cout << "file all diveces" << std::endl;
                        error_run_state = error_reconnect;
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                }
                break;
                case error_reconnect:
                {
                    std::cerr << "\033[1;31mreconnect start \033[0m" << std::endl;
                    this->init_ser();
                    auto it = robot_params.CANboards.begin();
                    for (size_t i = 1; i <= CANboard_num; i++, it++)
                    {
                        CANboards.push_back(canboard(i, &ser, it->second, canport_error_output_flag));
                    }

                    for (canboard &cb : CANboards)
                    {
                        cb.push_CANport(&CANPorts);
                    }
                    for (canport *cp : CANPorts)
                    {
                        // std::thread(&canport::send, &cp);
                        cp->puch_motor(&Motors);
                    }
                    set_port_motor_num(); // 设置通道上挂载的电机数，并获取主控板固件版本号
                    check_motor_connection_version();  // 检测电机连接是否正常
                    error_run_state = error_check;
                    std::cerr << "\033[1;31mreconnect end\033[0m" << std::endl;
                }
                break;
                default:
                break;
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::unique_lock<std::mutex> lock(error_check_mutex);
            if (error_check_cv.wait_for(lock, std::chrono::milliseconds(1000), 
                [this]{ return !error_check_flag; })) {
                // 收到退出信号，立即退出
                break;
            }
            // only state chaged, print state.
            if(error_run_state != last_error_run_state)
            {
                last_error_run_state = error_run_state;
                std::cout << "error_run_state = " << error_run_state << std::endl;
            }
        }
    }

    int robot::check_serial_dev_exist(int file_num)
    {
        int exist_num = 0;
        std::cout << "check serial dev exist" << std::endl;
        std::vector<std::string> dev_vec;
        for (size_t i = 0; i < file_num; i++)
        {
            std::string _dev = std::string("/dev/ttyACM") + std::to_string(i);
            std::cout << "check: " << _dev << std::endl;
            dev_vec.push_back(_dev);
        }
        for (auto &dev : dev_vec)
        {
            if(access(dev.c_str(),F_OK) == 0)
            {
                exist_num++;
                std::cout << "exist: " << dev << std::endl;
            }
        }
        std::cout << "exist_num = " << exist_num << std::endl;
        return exist_num;
    }


    /**
     * @brief 设置每个通道的电机数量，并查询主控板固件版本
     */
    void robot::set_port_motor_num()
    {
        for (canboard &cb : CANboards)
        {
            slave_v = cb.set_port_motor_num();
        }
    }
    
    
    void robot::send_get_motor_state_cmd()
    {
        if (fun_v >= fun_v4)
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd2();
            }
        }
        else if (fun_v >= fun_v2)
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd();
            }
        }
        else
        {
            for (motor *m : Motors)
            {
                m->velocity(0.0f);
            }
            motor_send_cmd();
        }
    }


    void robot::send_get_motor_version_cmd()
    {
        if (slave_v < COMBINE_VERSION(4, 0, 0))
        {
            std::cerr << "\033[1;31m << The current communication board does not support this function!!! << \033[0m" << std::endl;
            exit(0);
        }

        for (canboard &cb : CANboards)
        {
            cb.send_get_motor_version_cmd();
        }
    }


    void robot::motor_version_detection()
    {
        uint16_t v_min = 0xFFFF;
        uint16_t v_max = 0;
        uint16_t i = 0;

        std::cout << "---------------motor version---------------------" << std::endl;
        for (motor *m : Motors)
        {
            const auto v = m->get_version();
            printf("motors[%02d]: id:%02d v%d.%d.%d\r\n", i++, v->id, v->major, v->minor, v->patch);
            const uint16_t v_new = COMBINE_VERSION(v->major, v->minor, v->patch);
            if (v_min > v_new && v_new != 0)
            {
                v_min = v_new;
            }

            if (v_max < v_new)
            {
                v_max = v_new;
            }
        }
        std::cout << "-------------------------------------------------" << std::endl;

        if (v_min >= COMBINE_VERSION(4, 4, 6))
        {
            fun_v = fun_v5;
        }
        else if (v_min >= COMBINE_VERSION(4, 2, 3))
        {
            fun_v = fun_v4;
        }
        else if (v_min >= COMBINE_VERSION(4, 2, 2))
        {
            fun_v = fun_v3;
        }
        else if (v_min >= COMBINE_VERSION(4, 2, 0))
        {
            fun_v = fun_v2;
        }
        else
        {
            fun_v = fun_v1;
        }

        for (canboard &cb : CANboards)
        {
            cb.set_fun_v(fun_v, v_min);
        }

        printf("fun_v = %d, motor_min_v = %d.%d.%d, motor_max_v = %d.%d.%d\n", fun_v, 
            GET_MAJOR_VERSION(v_min), GET_MINOR_VERSION(v_min), GET_PATCH_VERSION(v_min),
            GET_MAJOR_VERSION(v_max), GET_MINOR_VERSION(v_max), GET_PATCH_VERSION(v_max));
        
        if (slave_v < COMBINE_VERSION(4, 7, 0) || v_max < COMBINE_VERSION(4, 6, 0))
        {
            return;
        }

        check_tqe_adjust_flag();
    }

    void robot::send_get_tqe_adjust_flag_cmd()
    {
        for (canboard &cb : CANboards)
        {
            cb.send_get_tqe_adjust_flag_cmd();
        }
    }


    void robot::check_tqe_adjust_flag()
    {
        int t = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;

        printf("Check the torque adjustment mark\n");
        while (t++ < 20)
        {
            send_get_tqe_adjust_flag_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                cdc_rx_motor_version_s* v = m->get_version();
                uint8_t tqe_flag = m->get_tqe_adjust_flag();

                if (COMBINE_VERSION(v->major, v->minor, v->patch) >= COMBINE_VERSION(4, 6, 0) && tqe_flag == 0xFF)
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
                else if (tqe_flag == 1)
                {
                    m->set_type(mNone);
                }
            }

            if (id.size() == 0)
            {
                break;
            }

            if (t % 5 == 0)
            {
                ROS_INFO(".");
            }
        }

        if (id.size() != 0)
        {
            for (int i = 0; i < id.size(); i++)
            {
                ROS_ERROR("CANboard(%d) CANport(%d) id(%d) Motor Failed to read torque adjust flag!!!", board[i], port[i], id[i]);
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }


    void robot::check_motor_connection_version()
    {
        int t = 0;
        int num = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;

        std::cout << "Detecting motor connection" << std::endl;
        while (t++ < 20)
        {
            send_get_motor_version_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            num = 0;
            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                cdc_rx_motor_version_s *v = m->get_version();
                if (v->major != 0)
                {
                    ++num;
                }
                else
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
            }

            if (num == Motors.size())
            {
                break;
            }

            if (t % 100 == 0)
            {
                std::cout << "." << std::endl;
            }
        }

        if (num == Motors.size())
        {
            std::cout << "\033[1;32mAll motor connections are normal\033[0m" << std::endl;
        }
        else
        {
            for (int i = 0; i < Motors.size() - num; i++)
            {
                std::cerr << "\033[1;31m" << "CANboard(" << board[i] << ") CANport(" << port[i] << ") id(" << id[i] << ") Motor connection disconnected!!!" << "\033[0m" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        motor_version_detection();
    }


    void robot::check_motor_connection_position()
    {
        int t = 0;
        int num = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;
        ROS_INFO("Detecting motor connection");
        while (t++ < 2000)
        {
            send_get_motor_state_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            num = 0;
            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                if (m->get_current_motor_state()->position != 999.0f)
                {
                    ++num;
                }
                else
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
            }

            if (num == Motors.size())
            {
                break;
            }

            if (t % 1000 == 0)
            {
                std::cout << "." << std::endl;
            }
        }

        if (num == Motors.size())
        {
            std::cout << "\033[1;32mAll motor connections are normal\033[0m" << std::endl;
        }
        else
        {
            for (int i = 0; i < Motors.size() - num; i++)
            {
                std::cerr << "\033[1;31m" << "CANboard(" << board[i] << ") CANport(" << port[i] << ") id(" << id[i] << ") Motor connection disconnected!!!" << "\033[0m" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    void robot::set_brake()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_brake();
        }
        motor_send_cmd();
    }

    void robot::set_stop()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_stop();
        }
        motor_send_cmd();
    }


    void robot::set_reset()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }


    void robot::set_reset_zero()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_reset_zero();
        }
    }


    void robot::set_reset_zero(std::initializer_list<int> motors)
    {
        for (auto const &motor : motors)
        {
            
            int board_id = Motors[motor]->get_motor_belong_canboard() - 1;
            int port_id = Motors[motor]->get_motor_belong_canport() - 1;
            int motor_id = Motors[motor]->get_motor_id();
            std::cout << board_id << ", " << port_id << ", " << motor_id << std::endl;

            set_reset();
            
            std::cout << "Motor " << motor << " settings have been successfully restored. Initiating zero position reset." << std::endl;
            if (CANPorts[port_id]->set_reset_zero(motor_id) == 0)
            {
                std::cout << "Motor " << motor << " reset to zero position successfully, awaiting settings save." << std::endl;
                if (CANPorts[port_id]->set_conf_write(motor_id) == 0)
                {
                    std::cout << "Motor " << motor << " settings saved successfully." << std::endl;
                }
                else
                {
                    std::cerr << "\033[1;31m << Motor " << motor << " settings saved failed. << \033[0m" << std::endl;
                }
            }
            else
            {
                std::cerr << "Motor " << motor << " reset to zero position failed." << std::endl;
            }
        }
    }


    void robot::set_timeout(int16_t t_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            for (canboard &cb : CANboards)
            {
                cb.set_time_out(t_ms);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void robot::set_timeout(uint8_t portx, int16_t t_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            CANboards[0].set_time_out(portx, t_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void robot::canboard_bootloader()
    {
        for (canboard &cb : CANboards)
        {
            cb.canboard_bootloader();
        }
    }

    void robot::canboard_fdcan_reset()
    {
        std::cout << "canboard fdcan reset" << std::endl;
        for (canboard &cb : CANboards)
        {
            cb.canboard_fdcan_reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
