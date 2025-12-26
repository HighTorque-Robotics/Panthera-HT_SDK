#include "robot.hpp"
#include <iostream>

#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>


std::atomic<bool> exitFlag(false);
void signalHandler(int signum) 
{
    exitFlag.store(true);
}


int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    hightorque_robot::robot rb;
    rb.lcm_enable();

    while(!exitFlag.load())
    {
        rb.send_get_motor_state_cmd();  // 发送查询电机状态指令（不控制电机）

        for (motor *m : rb.Motors)
        {
            motor_back_t motor = *m->get_current_motor_state();  // 从缓存中获取电机状态
            printf("ID: %2d, mode: %2d, fluat: %2d, pos: %2.3f, vel: %2.3f, tor: %2.3f\n", motor.ID, motor.mode, motor.fault, motor.position, motor.velocity, motor.torque);
            // printf(".2f  ", motor.position);
        }
        // printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

