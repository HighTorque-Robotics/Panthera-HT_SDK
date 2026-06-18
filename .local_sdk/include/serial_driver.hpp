#ifndef _serial_driver_H_
#define _serial_driver_H_


#include <serial/serial.h>
#include "serial_struct.hpp"
#include "motor.hpp"
#include <unordered_set>
#include <map>

class serial_driver
{
private:
    serial::Serial _ser;
    bool init_flag;
    std::map<int, motor *> Map_Motors_p;
    uint16_t *p_port_version = NULL;
    std::unordered_set<int> *p_motor_id = NULL;
    int *p_mode_flag = NULL;
    fun_version *p_fun_v = NULL;
    cdc_rx_fdcan_state_s *p_fdcan_state = NULL;
    bool canport_error_output_flag = false;

public:
    serial_driver(std::string *port, uint32_t baudrate, bool _canport_error_output_flag);
    ~serial_driver();

    bool error_flag;
    void send_2(cdc_tr_message_s *cdc_tr_message);
    uint16_t return_data_len(uint8_t cmd_id);
    void recv_1for6_42();
    void port_version_init(uint16_t *p);
    void port_motors_id_init(std::unordered_set<int> *_p_motor_id, int *_p_mode_flag);
    void init_map_motor(std::map<int, motor *> *_Map_Motors_p);
    void port_fun_v_init(fun_version *_p_fun_v);
    void port_fdcan_state_init(cdc_rx_fdcan_state_s *_p_fdcan_state);
    serial_driver(const serial_driver &) = delete;
    serial_driver &operator=(const serial_driver &) = delete;
    bool is_serial_error(void);
    void set_run_flag(bool flag);
    bool get_run_flag(void);
    void close(void);
};

#endif