#include "canport.hpp"
#include <chrono>
#include <thread>


canport::canport(int _CANport_num, int _CANboard_num, serial_driver *_ser, CANPortParams &canport_params, bool _canport_error_output_flag) : ser(_ser)
{
    canboard_id = _CANboard_num;
    canport_id = _CANport_num;
    motor_num = canport_params.motor_num;
    canport_error_output_flag = _canport_error_output_flag;

    if (PORT_MOTOR_NUM_MAX < motor_num)
    {
        std::cerr << "\033[1;31m" << "Too many motors, Supports up to " << PORT_MOTOR_NUM_MAX << " motors, but there are actually " << motor_num << " motors" << "\033[0m" << std::endl;
        exit(-1);
    }        

    for(auto motor_params : canport_params.motors)
    {
        port_motor_id.push_back(motor_params.second.id);
        if (id_max < motor_params.second.id)
        {
            id_max = motor_params.second.id;
        }
    }
    auto it = canport_params.motors.begin();
    for (size_t i = 1; i <= motor_num; i++, it++)
    {
        Motors.push_back(new motor(i, _CANport_num, _CANboard_num, &cdc_tr_message, id_max, it->second));
    }
    for (motor *m : Motors)
    {
        Map_Motors_p.insert(std::pair<int, motor *>(m->get_motor_id(), m));
    }
    ser->init_map_motor(&Map_Motors_p);
    ser->port_version_init(&port_version);
    ser->port_motors_id_init(&motors_id, &mode_flag);
    ser->port_fun_v_init(&fun_v);
    ser->port_fdcan_state_init(&fdcan_state);
}


uint16_t canport::set_motor_num()
{
    if (cdc_tr_message.head.s.cmd != MODE_SET_NUM)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_SET_NUM;
        cdc_tr_message.head.s.len = 2;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }

    cdc_tr_message.data.data[0] = motor_num;
    
    int t = 0;
    #define MAX_DALAY 1000  // 单位ms
    while (t++ < MAX_DALAY)
    {
        motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (port_version >= COMBINE_VERSION(3, 0, 0))
        {
            // printf("\033[1;32m ttt %d\033[0m", t);
            break;
        }
    }

    if (t < MAX_DALAY)
    {
        std::cout << "\033[1;32mCANboard(" << canboard_id << ") version is: v" << GET_MAJOR_VERSION(port_version) 
        << '.' << GET_MINOR_VERSION(port_version) << "." << GET_PATCH_VERSION(port_version) << "\033[0m" << std::endl;
    }
    else
    {
        std::cerr << "\033[1;31m" << "CANboard(" << canboard_id << ") CANport(" << canport_id << ") Connection disconnected!!!" << "\033[0m" << std::endl;
    }

    return port_version;
}


int canport::set_reset_zero()
{
    if (cdc_tr_message.head.s.cmd != MODE_RESET_ZERO)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RESET_ZERO;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    int t = 0;
    int num = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        num = 0;
        if (mode_flag == MODE_RESET_ZERO)
        {
            for (int i = 1; i <= motor_num; i++)
            {
                // if (motors_id.count(i) == 1)
                int id = port_motor_id[i - 1];
                if (motors_id.count(id) == 1)
                {
                    ++num;
                }
            }
        }

        if (num == motor_num)
        {
            break;
        }
    }

    if (num == motor_num)
    {
        std::cout << "\033[1;32mMotor zero position reset successfully, waiting for the motor to save the settings.\033[0m" << std::endl;
        return 0;
    }
    else 
    {
        std::cerr << "\033[1;31m" << "Motor reset to zero position failed." << "\033[0m" << std::endl;
        return 1;
    }
}


int canport::set_reset_zero(int id)
{
    
    if (cdc_tr_message.head.s.cmd != MODE_RESET_ZERO)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RESET_ZERO;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = id;

    int t = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (mode_flag == MODE_RESET_ZERO && motors_id.count(id) == 1)
        {
            return 0;
        }
    }

    return 1;
}


void canport::set_brake()
{
    if (cdc_tr_message.head.s.cmd != MODE_BRAKE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_BRAKE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_cmd();
    motor_send_cmd();
    motor_send_cmd();
}


void canport::set_stop()
{
    if (cdc_tr_message.head.s.cmd != MODE_STOP)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_STOP;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_cmd();
    motor_send_cmd();
    motor_send_cmd();
}


void canport::set_reset()
{
    if (cdc_tr_message.head.s.cmd != MODE_RESET)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_RESET;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    motor_send_cmd();
    motor_send_cmd();
    motor_send_cmd();
}


void canport::send_get_tqe_adjust_flag_cmd()
{
    if (cdc_tr_message.head.s.cmd != MODE_TQE_ADJS_FLAG)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_TQE_ADJS_FLAG;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_cmd();
    motor_send_cmd();
    motor_send_cmd();
}


void canport::set_conf_write()
{
    
    if (cdc_tr_message.head.s.cmd != MODE_CONF_WRITE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_CONF_WRITE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;

    int t = 0;
    int num = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        num = 0;
        if (mode_flag == MODE_CONF_WRITE)
        {
            for (int i = 1; i <= motor_num; i++)
            {
                int id = port_motor_id[i - 1];
                if (motors_id.count(id) == 1)
                {
                    ++num;
                }
            }
        }

        if (num == motor_num)
        {
            break;
        }
    }

    if (num == motor_num)
    {
        std::cout << "\033[1;32mSettings saved successfully.\033[0m" << std::endl;
    }
    else 
    {
        std::cerr << "\033[1;31m" << "Failed to save settings." << "\033[0m" << std::endl;
        exit(0);
    }
}


int canport::set_conf_write(int id)
{
    
    if (cdc_tr_message.head.s.cmd != MODE_CONF_WRITE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_CONF_WRITE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = id;

    int t = 0;
    int max_delay = 10000;
    motors_id.clear();
    mode_flag = 0;
    while (t++ < max_delay)
    {
        motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (mode_flag == MODE_CONF_WRITE && motors_id.count(id) == 1)
        {
            return 0;
        }
    }

    return 1;
}


void canport::send_get_motor_state_cmd()
{
    if (cdc_tr_message.head.s.cmd != MODE_MOTOR_STATE)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_MOTOR_STATE;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_cmd();
}


void canport::send_get_motor_state_cmd2()
{
    if (cdc_tr_message.head.s.cmd != MODE_MOTOR_STATE2)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_MOTOR_STATE2;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_cmd();
}


void canport::send_get_motor_version_cmd()
{
    if (cdc_tr_message.head.s.cmd != MODE_MOTOR_VERSION)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_MOTOR_VERSION;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = 0x7f;
    motor_send_cmd();
}


void canport::set_fun_v(fun_version v, uint16_t motor_version)
{
    if (cdc_tr_message.head.s.cmd != MODE_FUN_V)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_FUN_V;
        if (canport_error_output_flag)
        {
            cdc_tr_message.head.s.len = 3;
        }
        else
        {
            cdc_tr_message.head.s.len = 1;
        }
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    cdc_tr_message.data.data[0] = v;
    *(uint16_t *)&cdc_tr_message.data.data[1] = motor_version;

    int t = 0;
    #define MAX_DALAY 1000  // 单位ms
    while (t++ < MAX_DALAY)
    {
        motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (v == fun_v)
        {
            // printf("\033[1;32m ttt %d\033[0m", t);
            break;
        }
    }

    if (t == MAX_DALAY)
    {
        std::cerr << "\033[1;31m" << "CANboard(" << canboard_id << ") CANport(" << canport_id << ") fun_v err!!!" << "\033[0m" << std::endl;
    }
}


void canport::set_time_out(int16_t t_ms)
{
    if (cdc_tr_message.head.s.cmd != MODE_TIME_OUT)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_TIME_OUT;
        cdc_tr_message.head.s.len = motor_num * 2;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }

    for (int i = 0; i < motor_num; i++)
    {
        cdc_tr_message.data.timeout[i] = t_ms;
    }
    
    motor_send_cmd();
}


void canport::puch_motor(std::vector<motor *> *_Motors)
{
    for (motor *m : Motors)
    {
        _Motors->push_back(m);
    }
}


void canport::motor_send_cmd()
{
    ser->send_2(&cdc_tr_message);
}


int canport::get_motor_num()
{
    return motor_num;
}


int canport::get_canboard_id()
{
    return canboard_id;
}


int canport::get_canport_id()
{
    return canport_id;
}


cdc_rx_fdcan_state_s &canport::get_canport_state()
{
    return fdcan_state;
}


void canport::canboard_bootloader()
{
    if (cdc_tr_message.head.s.cmd != MODE_BOOTLOADER)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_BOOTLOADER;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    
    motor_send_cmd();
    motor_send_cmd();
    motor_send_cmd();
}


void canport::canboard_fdcan_reset()
{
    if (cdc_tr_message.head.s.cmd != MODE_FDCAN_RESET)
    {
        cdc_tr_message.head.s.head = 0XF7;
        cdc_tr_message.head.s.cmd = MODE_FDCAN_RESET;
        cdc_tr_message.head.s.len = 1;
        memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
    }
    
    motor_send_cmd();
    motor_send_cmd();
    motor_send_cmd();
}
