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
    int cont = 0;

    while(!exitFlag.load())
    {
        int i = 0;
        for (motor *m : rb.Motors)
        {
            motor_back_t motor = *m->get_current_motor_state();  // 从缓存中获取电机状态
            printf("ID: %2d, mode: %2d, fluat: %2d, pos: %2.3f, vel: %2.3f, tor: %2.3f\n", motor.ID, motor.mode, motor.fault, motor.position, motor.velocity, motor.torque);

            m->pos_vel_MAXtqe(0, 0.3, 100);  // 包含查询电机状态指令
            // printf(".2f  ", motor.position);
        }
        // printf("\n");
        rb.motor_send_cmd();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}