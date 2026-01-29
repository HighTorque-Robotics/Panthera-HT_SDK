#include "motor.hpp"
#include <iostream>




motor::motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max, MotorParams &motor_params)
: CANport_num(_CANport_num), CANboard_num(_CANboard_num), p_cdc_tx_message(_p_cdc_tx_message), id_max(_id_max)
{
    motor_name = motor_params.name;
    id = motor_params.id;
    num = motor_params.num;
    pos_limit_enable = motor_params.pos_limit_enable;
    pos_upper = motor_params.pos_upper;
    pos_lower = motor_params.pos_lower;
    tor_limit_enable = motor_params.tor_limit_enable;
    tor_upper = motor_params.tor_upper;
    tor_lower = motor_params.tor_lower;


    set_motor_type(motor_params.type);
    data.time = 0;
    data.ID = id;
    data.mode = 0;
    data.fault = 0;
    data.position = 999.0f;
    data.velocity = 0;
    data.torque = 0;
    data.num = 0;
}


inline int16_t motor::int16_limit(int32_t data)
{
    if (data >= 32700)
    {
        std::cout << "\033[1;32mPID output has reached the saturation limit.\033[0m" << std::endl;
        return (int16_t)32700;
    }
    else if (data <= -32700)
    {
        std::cout << "\033[1;32mPID output has reached the saturation limit.\033[0m" << std::endl;
        return (int16_t)-32700;
    }

    return (int16_t)data;
}


inline int16_t motor::pos_float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return int16_limit(in_data / my_2pi * 10000.0);
    case (pos_vel_convert_type::angle_360):
        return int16_limit(in_data / 360.0 * 10000.0);
    case (pos_vel_convert_type::turns):
        return int16_limit(in_data * 10000.0);
    default:
        return int16_t();
    }
}

inline int16_t motor::vel_float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return int16_limit(in_data / my_2pi * 4000.0);
    case (pos_vel_convert_type::angle_360):
        return int16_limit(in_data / 360.0 * 4000.0);
    case (pos_vel_convert_type::turns):
        return int16_limit(in_data * 4000.0);
    default:
        return int16_t();
    }
}

inline int16_t motor::tqe_float2int(float in_data, motor_type motor_type)
{
    auto it = motor_tqe_adj.find(motor_type);
    if (it == motor_tqe_adj.end())
    {
        ROS_ERROR("Motor Type SettingMotor type setting error");
        exit(0);
        return int16_t();
    }

    return int16_limit(in_data / (it->second * 0.01f));
}


inline float motor::tqe_int2float(int16_t in_data, motor_type motor_type)
{
    auto it = motor_tqe_adj.find(motor_type);
    if (it == motor_tqe_adj.end())
    {
        ROS_ERROR("Motor Type SettingMotor type setting error");
        exit(0);
        return int16_t();
    }

    return (in_data * (it->second * 0.01f));
}


inline float motor::pid_scale(float in_data, motor_type motor_type)
{
    auto it = motor_tqe_adj.find(motor_type);
    if (it == motor_tqe_adj.end())
    {
        ROS_ERROR("Motor Type SettingMotor type setting error");
        exit(0);
        return int16_t();
    }

    return (in_data / it->second);
}


inline int16_t motor::kp_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        tqe = (int32_t)(in_data * 10 * my_2pi);
        break;
    case (pos_vel_convert_type::angle_360):
        tqe = (int32_t)(in_data * 10 * 360);
        break;
    case (pos_vel_convert_type::turns):
        tqe = (int32_t)(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }
    return int16_limit(tqe);
}


inline int16_t motor::ki_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        tqe = (int32_t)(in_data * 10 * my_2pi);
        break;
    case (pos_vel_convert_type::angle_360):
        tqe = (int32_t)(in_data * 10 * 360);
        break;
    case (pos_vel_convert_type::turns):
        tqe = (int32_t)(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }

    return int16_limit(tqe);
}

inline int16_t motor::kd_float2int(float in_data, uint8_t type, motor_type motor_type)
{
    in_data = pid_scale(in_data, motor_type);
    
    int32_t tqe = 0;
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        tqe = (int32_t)(in_data * 10 * my_2pi);
        break;
    case (pos_vel_convert_type::angle_360):
        tqe = (int32_t)(in_data * 10 * 360);
        break;
    case (pos_vel_convert_type::turns):
        tqe = (int32_t)(in_data * 10);
        break;
    default:
        tqe = int16_t();
        break;
    }

    return int16_limit(tqe);
}

inline float motor::pos_int2float(int16_t in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return (float)(in_data * my_2pi / 10000.0);
    case (pos_vel_convert_type::angle_360):
        return (float)(in_data * 360.0 / 10000.0);
    case (pos_vel_convert_type::turns):
        return (float)(in_data / 10000.0);
    default:
        return float();
    }
}

inline float motor::vel_int2float(int16_t in_data, uint8_t type)
{
    switch (type)
    {
    case (pos_vel_convert_type::radian_2pi):
        return (float)(in_data * my_2pi / 4000.0);
    case (pos_vel_convert_type::angle_360):
        return (float)(in_data * 360.0 / 4000.0);
    case (pos_vel_convert_type::turns):
        return (float)(in_data / 4000.0);
    default:
        return float();
    }
}

uint16_t motor::get_data_len(uint8_t mode, uint16_t num)
{
    uint8_t motor_one_len = 0;
    switch (mode)
    {
        case MODE_POSITION:
        case MODE_VELOCITY:
        case MODE_TORQUE:
        case MODE_VOLTAGE:
        case MODE_CURRENT:
        case MODE_TIME_OUT:
            motor_one_len = 2;
            break;
        case MODE_POS_VEL_TQE:
        case MODE_POS_VEL_ACC:
            motor_one_len = 6;
            break;
        case MODE_POS_VEL_KP_KD:
            motor_one_len = 8;
            break;
        case MODE_POS_VEL_TQE_KP_KD_2:
            motor_one_len = 10;
            break;
        default:
            motor_one_len = 0;
            ROS_ERROR("This mode has beenThis mode is deprecated.");
            exit(0);
    }

    uint8_t fdcan_one_len = 60;
    if (mode == MODE_POS_VEL_KP_KD)
    {
        fdcan_one_len = 56;
    }

    const uint16_t len = motor_one_len * num;
    const uint8_t mul = len / fdcan_one_len;
    const uint8_t rem = len % fdcan_one_len;
    uint8_t rem_len = 0;

    if (rem <= 6)
    {
        rem_len = rem;
    }
    else if (rem <= 10)
    {
        rem_len = 10;
    }
    else if (rem <= 14)
    {
        rem_len = 14;
    }
    else if (rem <= 18)
    {
        rem_len = 18;
    }
    else if (rem <= 22)
    {
        rem_len = 22;
    }
    else if (rem <= 30)
    {
        rem_len = 30;
    }
    else if (rem <= 46)
    {
        rem_len = 46;
    }
    else 
    {
        rem_len = fdcan_one_len;
    }

    return mul * fdcan_one_len + rem_len;
}

void motor::position(float position)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POSITION)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POSITION;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_POSITION, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.position[MEM_INDEX_ID(id)] = pos_float2int(position, pos_vel_type);
}

void motor::velocity(float velocity)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_VELOCITY)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_VELOCITY;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_VELOCITY, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.velocity[MEM_INDEX_ID(id)] = vel_float2int(velocity, pos_vel_type);
}

void motor::torque(float torque)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_TORQUE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_TORQUE;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_TORQUE, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.torque[MEM_INDEX_ID(id)] = tqe_float2int(torque, type_);
}

void motor::voltage(float voltage)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_VOLTAGE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_VOLTAGE;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_VOLTAGE, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.voltage[MEM_INDEX_ID(id)] = int16_limit(voltage * 10);
}

void motor::current(float current)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_CURRENT)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_CURRENT;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_CURRENT, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.current[MEM_INDEX_ID(id)] = int16_limit(current * 10);
}

void motor::set_motorout(int16_t t_ms)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_TIME_OUT)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_TIME_OUT;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_TIME_OUT, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }

    p_cdc_tx_message->data.timeout[MEM_INDEX_ID(id)] = t_ms;
}

void motor::pos_vel_MAXtqe(float position, float velocity, float torque_max)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_TQE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_TQE;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_POS_VEL_TQE, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }
    p_cdc_tx_message->data.pos_vel_tqe[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_tqe[MEM_INDEX_ID(id)].vel = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_tqe[MEM_INDEX_ID(id)].tqe = tqe_float2int(torque_max, type_);
}


void motor::pos_vel_acc(float position, float velocity, float acc)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_ACC)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_ACC;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_POS_VEL_ACC, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }
    p_cdc_tx_message->data.pos_vel_acc[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_acc[MEM_INDEX_ID(id)].vel = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_acc[MEM_INDEX_ID(id)].acc = int16_limit(acc * 1000);
}


void motor::pos_vel_tqe_kp_kd(float position, float velocity, float torque, float kp, float kd)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_TQE_KP_KD_2)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_TQE_KP_KD_2;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_POS_VEL_TQE_KP_KD_2, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }
    p_cdc_tx_message->data.pos_vel_tqe_kp_kd[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_tqe_kp_kd[MEM_INDEX_ID(id)].vel = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_tqe_kp_kd[MEM_INDEX_ID(id)].tqe = tqe_float2int(torque, type_);
    p_cdc_tx_message->data.pos_vel_tqe_kp_kd[MEM_INDEX_ID(id)].kp = kp_float2int(kp, pos_vel_type, type_); 
    p_cdc_tx_message->data.pos_vel_tqe_kp_kd[MEM_INDEX_ID(id)].kd = kd_float2int(kd, pos_vel_type, type_);
}


void motor::pos_vel_kp_kd(float position, float velocity, float kp, float kd)
{
    if (p_cdc_tx_message->head.s.cmd != MODE_POS_VEL_KP_KD)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_POS_VEL_KP_KD;
        p_cdc_tx_message->head.s.len = get_data_len(MODE_POS_VEL_KP_KD, id_max);
        for (uint8_t i = 0; i < p_cdc_tx_message->head.s.len / sizeof(int16_t); i++)
        {
            p_cdc_tx_message->data.data16[i] = 0x8000;
        }
    }
    p_cdc_tx_message->data.pos_vel_kp_kd[MEM_INDEX_ID(id)].pos = pos_float2int(position, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_kp_kd[MEM_INDEX_ID(id)].vel = vel_float2int(velocity, pos_vel_type);
    p_cdc_tx_message->data.pos_vel_kp_kd[MEM_INDEX_ID(id)].kp = kp_float2int(kp, pos_vel_type, type_);  
    p_cdc_tx_message->data.pos_vel_kp_kd[MEM_INDEX_ID(id)].kd = kd_float2int(kd, pos_vel_type, type_); 
}


void motor::stop()
{
    if (p_cdc_tx_message->head.s.cmd != MODE_STOP)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_STOP;
        p_cdc_tx_message->head.s.len = id_max * sizeof(uint8_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.data[i] = 0;
        }
    }

    p_cdc_tx_message->data.data[MEM_INDEX_ID(id)] = 1;
}


void motor::brake()
{
    if (p_cdc_tx_message->head.s.cmd != MODE_BRAKE)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_BRAKE;
        p_cdc_tx_message->head.s.len = id_max * sizeof(uint8_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.data[i] = 0;
        }
    }

    p_cdc_tx_message->data.data[MEM_INDEX_ID(id)] = 1;
}


void motor::reset()
{
    if (p_cdc_tx_message->head.s.cmd != MODE_RESET)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_RESET;
        p_cdc_tx_message->head.s.len = id_max * sizeof(uint8_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.data[i] = 0;
        }
    }

    p_cdc_tx_message->data.data[MEM_INDEX_ID(id)] = 1;
}


/**
 * @brief 仅发送查询电机状态指令
 */
void motor::send_state_cmd()
{
    if (p_cdc_tx_message->head.s.cmd != MODE_MOTOR_STATE2)
    {
        p_cdc_tx_message->head.s.head = 0xF7;
        p_cdc_tx_message->head.s.cmd = MODE_MOTOR_STATE2;
        p_cdc_tx_message->head.s.len = id_max * sizeof(uint8_t);
        for (uint8_t i = 0; i < id_max; i++)
        {
            p_cdc_tx_message->data.data[i] = 0;
        }
    }

    p_cdc_tx_message->data.data[MEM_INDEX_ID(id)] = 1;
}


#include <chrono>
void motor::fresh_data(uint8_t mode, uint8_t fault, int16_t position, int16_t velocity, int16_t torque)
{
    data.num++;
    data.mode = mode;
    data.fault = fault;
    data.position = pos_int2float(position, pos_vel_type);
    data.velocity = vel_int2float(velocity, pos_vel_type);
    data.torque = tqe_int2float(torque, type_);

    auto now = std::chrono::system_clock::now();
    data.time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;

    if(pos_limit_enable)
    {
        // 判断是否超过电机限制角度
        if(data.position > pos_upper)
        {
            ROS_ERROR("Motor %d exceed position upper limit.", id);
            pos_limit_flag = 1;
        }
        else if(data.position < pos_lower)
        {
            ROS_ERROR("Motor %d exceed position lower limit.", id);
            pos_limit_flag = -1;
        }
    }
    
    if(tor_limit_enable)
    {
        // 判断是否超过电机扭矩限制
        if(data.torque > tor_upper)
        {
            ROS_ERROR("Motor %d exceed torque upper limit.", id);
            tor_limit_flag = 1;
        }
        else if(data.torque < tor_lower)
        {
            ROS_ERROR("Motor %d exceed torque lower limit.", id);
            tor_limit_flag = -1;
        }
    }
}


void motor::set_motor_type(std::string type_str)
{
    try 
    {
        type_ = motor_type2.at(type_str);
    } 
    catch (const std::out_of_range& e) 
    {
        ROS_ERROR("Motor model error: %s", type_str.c_str());

        std::cout << "----------------Supported motor models are:-------------------" << std::endl;
        for (auto it = motor_type2.begin(); it != motor_type2.end(); ++it)
        {
            std::cout << it->first << std::endl;
        }
        std::cout << "--------------------------------------------------------------" << std::endl;
        exit(-1); 
    }
}


int motor::get_motor_id() 
{ 
    return id; 
}


int motor::get_motor_type() 
{ 
    return type_; 
}


motor_type motor::get_motor_enum_type()
{ 
    return type_; 
}


int motor::get_motor_num() 
{ 
    return num; 
}


int motor::get_motor_belong_canport() 
{ 
    return CANport_num; 
}


int motor::get_motor_belong_canboard() 
{ 
    return CANboard_num; 
}

motor_pos_vel_tqe_kp_kd_s* motor::return_pos_vel_tqe_kp_kd_p()
{
    return &cmd_int16_5param;
}

size_t motor::return_size_motor_pos_vel_tqe_kp_kd_s()
{
    return sizeof(motor_pos_vel_tqe_kp_kd_s);
}

motor_back_t* motor::get_current_motor_state()
{
    return &data;
}

std::string motor::get_motor_name()
{
    return motor_name;
}


void motor::set_version(cdc_rx_motor_version_s &v)
{
    version.id = v.id;
    version.major = v.major;
    version.minor = v.minor;
    version.patch = v.patch;

    // ROS_INFO("ID: %d, version = %d.%d.%d", version.id, version.major, version.minor, version.patch);
}


cdc_rx_motor_version_s* motor::get_version()
{
    return &version;
}


void motor::print_version()
{
    ROS_INFO("ID: %d, version = %d.%d.%d", version.id, version.major, version.minor, version.patch);
}


void motor::set_type(motor_type t)
{
    type_ = t;
}


void motor::set_tqe_adjust_flag(uint8_t flag)
{
    tqe_adjust_flag = flag;
}


uint8_t motor::get_tqe_adjust_flag()
{
    return tqe_adjust_flag;
}


void motor::set_num()
{
    data.num = 0;
}
