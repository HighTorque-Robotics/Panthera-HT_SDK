#include "serial_driver.hpp"
#include <iostream>

serial_driver::serial_driver(std::string *port, uint32_t baudrate, bool _canport_error_output_flag): canport_error_output_flag(_canport_error_output_flag)
{
    init_flag = false;
    error_flag = false;
    _ser.setPort(*port); // 设置打开的串口名称
    _ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
    _ser.setTimeout(to);                                       // 设置串口的timeout

    // 打开串口
    try
    {
        _ser.open(); // 打开串口
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m" << "Motor Unable to open port" << "\033[0m" << std::endl;
        this->error_flag = true;
    }
    if (_ser.isOpen())
    {
        // std::cout << "\033[1;32mMotor Serial Port initialized.\033[0m" << std::endl; // 成功打开串口，打印信息
        init_flag = true;
    }
    else
    {
    }
    init_flag = true;
}


serial_driver::~serial_driver()
{
    if(_ser.isOpen())
    {
        _ser.close();
        init_flag = false;
    }
}


void serial_driver::recv_1for6_42()
{
    uint8_t CRC8 = 0;
    uint16_t CRC16 = 0;
    cdc_tr_message_data_s cdc_rx_message_data = {0};
    while (init_flag)
    {
        cdc_tr_message_head_data_s SOF = {0};
        try
        {
            _ser.read(&(SOF.head), 1); 
            if (SOF.head == 0xF7)      //  head
            {
                _ser.read(&(SOF.cmd), 4);
                if (SOF.crc8 == Get_CRC8_Check_Sum((uint8_t *)&(SOF.cmd), 3, 0xFF)) // cmd_id
                {
                    _ser.read((uint8_t *)&CRC16, 2);
                    _ser.read((uint8_t *)&cdc_rx_message_data, SOF.len);
                    if (CRC16 != crc_ccitt(0xFFFF, (const uint8_t *)&cdc_rx_message_data, SOF.len))
                    {
                        memset(&cdc_rx_message_data, 0, sizeof(cdc_rx_message_data) / sizeof(int));
                    }
                    else
                    {
                        // printf("cmd %02X  ", SOF.cmd);
                        // for (int i = 0; i < SOF.len; i++)
                        // {
                        //     printf("0x%02X ", cdc_rx_message_data.data[i]);
                        // }
                        // printf("\n");

                        switch (SOF.cmd)
                        {
                        case (MODE_RESET_ZERO):
                        case (MODE_CONF_WRITE):
                            *p_mode_flag = SOF.cmd;
                            for (int i = 0; i < SOF.len; i++)
                            {
                                p_motor_id->insert(cdc_rx_message_data.data[i]);
                            }
                            break;

                        case(MODE_SET_NUM):
                            {
                                const uint8_t v_major = cdc_rx_message_data.data[2];
                                const uint8_t v_minor = cdc_rx_message_data.data[3];
                                const uint8_t v_patch = SOF.len == 4 ? 0 : cdc_rx_message_data.data[4];
                                *p_port_version = COMBINE_VERSION(v_major, v_minor, v_patch);
                            }
                            break;
                        case(MODE_FUN_V):
                            *p_fun_v = (fun_version)cdc_rx_message_data.data[0];
                            break;
                        case(MODE_MOTOR_VERSION):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_version_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_version[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->set_version(cdc_rx_message_data.motor_version[i]);
                                }
                            }
                            break;

                        case(MODE_MOTOR_STATE):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_state_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(0, 0, 
                                                    cdc_rx_message_data.motor_state[i].pos,
                                                    cdc_rx_message_data.motor_state[i].vel,
                                                    cdc_rx_message_data.motor_state[i].tqe);
                                }
                            }
                            break;
                        case(MODE_MOTOR_STATE2):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_state2_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state2[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(
                                                    cdc_rx_message_data.motor_state2[i].mode,
                                                    cdc_rx_message_data.motor_state2[i].fault,
                                                    cdc_rx_message_data.motor_state2[i].pos,
                                                    cdc_rx_message_data.motor_state2[i].vel,
                                                    cdc_rx_message_data.motor_state2[i].tqe);
                                }
                            }
                            break;
                        case(MODE_FDCAN_MOTOR_STATE2):
                            {
                                cdc_rx_fdcan_state_s &_p_fdcan_state = cdc_rx_message_data.fdcan_motor_state.fdcan_state;
                                
                                if (canport_error_output_flag)
                                {
                                    if (_p_fdcan_state.fault > FDCAN_STATUS_ERROR_WARNING || _p_fdcan_state.fault == FDCAN_STATUS_UNKNOWN)
                                    {
                                        ROS_ERROR("canport[%d] flaut = %d, rx = %d, tx = %d", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                    else if (_p_fdcan_state.fault == FDCAN_STATUS_ERROR_WARNING)
                                    {
                                        ROS_INFO("\033[1;32mcanport[%d] flaut = %d, rx = %d, tx = %d\033[0m", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                }
                                
                                p_fdcan_state->fault = _p_fdcan_state.fault;
                                p_fdcan_state->rx_err_num = _p_fdcan_state.rx_err_num;
                                p_fdcan_state->tx_err_num = _p_fdcan_state.tx_err_num;
                                

                                for (size_t i = 0; i < (SOF.len - sizeof(cdc_rx_fdcan_state_s)) / sizeof(cdc_rx_motor_state2_s); i++)
                                {
                                    auto it = Map_Motors_p.find(cdc_rx_message_data.fdcan_motor_state.motor_state2[i].id);
                                    if (it != Map_Motors_p.end())
                                    {
                                        it->second->fresh_data(
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].mode,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].fault,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].pos,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].vel,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].tqe);
                                    }
                                }
                            }
                            break;
                        case(MODE_FDCAN_MOTOR_STATE):
                            {
                                cdc_rx_fdcan_state_s &_p_fdcan_state = cdc_rx_message_data.fdcan_motor_state.fdcan_state;
                                
                                if (canport_error_output_flag)
                                {
                                    if (_p_fdcan_state.fault > FDCAN_STATUS_ERROR_WARNING || _p_fdcan_state.fault == FDCAN_STATUS_UNKNOWN)
                                    {
                                        ROS_ERROR("canport[%d] flaut = %d, rx = %d, tx = %d", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                    else if (_p_fdcan_state.fault == FDCAN_STATUS_ERROR_WARNING)
                                    {
                                        ROS_INFO("\033[1;32mcanport[%d] flaut = %d, rx = %d, tx = %d\033[0m", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                }
                                
                                p_fdcan_state->fault = _p_fdcan_state.fault;
                                p_fdcan_state->rx_err_num = _p_fdcan_state.rx_err_num;
                                p_fdcan_state->tx_err_num = _p_fdcan_state.tx_err_num;
                                

                                for (size_t i = 0; i < (SOF.len - sizeof(cdc_rx_fdcan_state_s)) / sizeof(cdc_rx_motor_state_s); i++)
                                {
                                    auto it = Map_Motors_p.find(cdc_rx_message_data.fdcan_motor_state.motor_state[i].id);
                                    if (it != Map_Motors_p.end())
                                    {
                                        it->second->fresh_data(0,0,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state[i].pos,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state[i].vel,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state[i].tqe);
                                    }
                                }
                            }
                            break;
                        case(MODE_TQE_ADJS_FLAG):
                            for (size_t i = 0; i < (SOF.len / sizeof(cdc_rx_motor_flag_s)); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_flag[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->set_tqe_adjust_flag(cdc_rx_message_data.motor_flag[i].flag);
                                }
                            }
                            break;
                        default:
                            break;
                        }
                    }
                }
                else
                {
                    // ROS_ERROR("clcl");
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << '\n';
            _ser.close();
            error_flag = true;
            break;
        }
    }
}

bool serial_driver::is_serial_error(void)
{
    return this->error_flag;
}

void serial_driver::set_run_flag(bool flag)
{
    this->init_flag = flag;
}

bool serial_driver::get_run_flag(void)
{
    return this->init_flag;
}

void serial_driver::close(void)
{
    try
    {
        /* code */
        if(_ser.isOpen())
        {
            _ser.flush();
        }
        _ser.close();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void serial_driver::send_2(cdc_tr_message_s *cdc_tr_message)
{
    cdc_tr_message->head.s.crc8 = Get_CRC8_Check_Sum(&(cdc_tr_message->head.data[1]), 3, 0xFF);
    cdc_tr_message->head.s.crc16 = crc_ccitt(0xFFFF, &(cdc_tr_message->data.data[0]), cdc_tr_message->head.s.len);

    // uint8_t *byte_ptr = (uint8_t *)&cdc_tr_message->head.s.head;
    // printf("send:\n");
    // for (size_t i = 0; i < cdc_tr_message->head.s.len + 7; i++)
    // {
    //     printf("0x%.2X ", byte_ptr[i]);
    // }
    // printf("\n\n");

    try
    {
        if(_ser.isOpen())
        {
            _ser.write((const uint8_t *)&cdc_tr_message->head.s.head, cdc_tr_message->head.s.len + sizeof(cdc_tr_message_head_s));
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        _ser.close();
        error_flag = true;
    }
}


void serial_driver::port_version_init(uint16_t *p)
{
    p_port_version = p;
}


void serial_driver::port_motors_id_init(std::unordered_set<int> *_p_motor_id, int *_p_mode_flag)
{
    p_motor_id = _p_motor_id;
    p_mode_flag = _p_mode_flag;
}


void serial_driver::init_map_motor(std::map<int, motor *> *_Map_Motors_p)
{
    Map_Motors_p = *_Map_Motors_p;
}


void serial_driver::port_fun_v_init(fun_version *_p_fun_v)
{
    p_fun_v = _p_fun_v;
}


void serial_driver::port_fdcan_state_init(cdc_rx_fdcan_state_s *_p_fdcan_state)
{
    p_fdcan_state = _p_fdcan_state;
    p_fdcan_state->fault = FDCAN_STATUS_OK;
    p_fdcan_state->rx_err_num = 0;
    p_fdcan_state->tx_err_num = 0;
}
