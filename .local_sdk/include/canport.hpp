#ifndef _CANPORT_H_
#define _CANPORT_H_
#include "motor.hpp"
#include <condition_variable>
#include <thread>
#include "serial_driver.hpp"
#include <unordered_set>
#include <iostream>
#include "parse_robot_params.hpp"


#define  PORT_MOTOR_NUM_MAX  30

class canport
{
private:
    int motor_num;
    std::vector<motor *> Motors;
    std::map<int, motor *> Map_Motors_p;
    int canboard_id, canport_id;
    serial_driver *ser;
    cdc_tr_message_s cdc_tr_message;
    int id_max = 0;
    uint16_t port_version = 0;
    fun_version fun_v = fun_v1;
    std::unordered_set<int> motors_id;
    int mode_flag = 0;
    std::vector<int> port_motor_id;
    std::vector<cdc_rx_motor_version_s *> motor_version;
    cdc_rx_fdcan_state_s fdcan_state;
    bool canport_error_output_flag = false;

public:
    canport(int _CANport_num, int _CANboard_num, serial_driver *_ser, CANPortParams& canport_params, bool _canport_error_output_flag);

    uint16_t set_motor_num();
    int set_reset_zero();
    int set_reset_zero(int id);
    void set_brake();
    void set_stop();
    void set_motor_runzero();
    void set_reset();
    void set_conf_write();
    int set_conf_write(int id);
    void send_get_motor_state_cmd();
    void send_get_motor_state_cmd2();
    void send_get_motor_version_cmd();
    void set_fun_v(fun_version v, uint16_t motor_version);
    void set_data_reset();
    void set_time_out(int16_t t_ms);
    void puch_motor(std::vector<motor *> *_Motors);
    void motor_send_cmd();
    int get_motor_num();
    int get_canboard_id();
    int get_canport_id();
    cdc_rx_fdcan_state_s &get_canport_state();
    void canboard_bootloader();
    void canboard_fdcan_reset();
    void send_get_tqe_adjust_flag_cmd();
};

#endif
