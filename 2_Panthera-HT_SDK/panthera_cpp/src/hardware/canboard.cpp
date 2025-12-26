#include "canboard.hpp"


canboard::canboard(int _CANboard_ID, std::vector<serial_driver *> *ser, CANBoardParams &canboard_params, bool _canport_error_output_flag)
{   
    auto it = canboard_params.CANports.begin();
    for (size_t j = 1; j <= canboard_params.CANport_num; j++, it++) // 一个串口对应一个CANport
    {
        CANport.push_back(new canport(j, _CANboard_ID, (*ser)[(_CANboard_ID - 1) * CANport_num + j - 1], it->second, _canport_error_output_flag));
    }
}


std::vector<canport*>& canboard::get_CANport()
{
    return CANport;
}


int canboard::get_CANport_num()
{
    return CANport_num;
}


void canboard::push_CANport(std::vector<canport*> *_CANport)
{
    for (canport *c : CANport)
    {
        _CANport->push_back(c);
    }
}


void canboard::motor_send_cmd()
{
    for (canport *c : CANport)
    {
        c->motor_send_cmd();
    }
}


void canboard::set_brake()
{
    for (canport *c : CANport)
    {
        c->set_brake();
    }
}


void canboard::set_stop()
{
    for (canport *c : CANport)
    {
        c->set_stop();
    }
}


void canboard::set_reset()
{
    for (canport *c : CANport)
    {
        c->set_reset();
    }
}


uint16_t canboard::set_port_motor_num()
{
    uint16_t v = 0;
    for (canport *c : CANport)
    {
        v = c->set_motor_num();
    }

    return v;
}


void canboard::send_get_motor_state_cmd()
{
    for (canport *c : CANport)
    {
        c->send_get_motor_state_cmd();
    }
}


void canboard::send_get_motor_state_cmd2()
{
    for (canport *c : CANport)
    {
        c->send_get_motor_state_cmd2();
    }
}


void canboard::send_get_motor_version_cmd()
{
    for (canport *c : CANport)
    {
        c->send_get_motor_version_cmd();
    }
}


void canboard::set_fun_v(fun_version v, uint16_t motor_version)
{
    for (canport *c : CANport)
    {
        c->set_fun_v(v, motor_version);
    }
}


void canboard::send_get_tqe_adjust_flag_cmd()
{
    for (canport *c : CANport)
    {
        c->send_get_tqe_adjust_flag_cmd();
    }
}


void canboard::set_reset_zero()
{
    for (canport *c : CANport)
    {
        for (int i = 0; i < 5; i++)
        {
            c->set_reset();
            c->motor_send_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (c->set_reset_zero() == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            c->set_conf_write();
        }
        c->set_reset();
        c->motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        c->motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


void canboard::set_time_out(int16_t t_ms)
{
    for (canport *c : CANport)
    {
        c->set_time_out(t_ms);
    }
}


void canboard::set_time_out(uint8_t portx, int16_t t_ms)
{
    CANport[portx]->set_time_out(t_ms);
}


void canboard::canboard_bootloader()
{
    CANport[0]->canboard_bootloader();
}

void canboard::canboard_fdcan_reset()
{
    for (canport *c : CANport)
    {
        c->canboard_fdcan_reset();
    }
}