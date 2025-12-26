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
    const int motor_num = rb.Motors.size();
    int cont = 0;
    float angle = 0.2;

    while(!exitFlag.load())
    {
        for (int i = 0; i < motor_num; i++)
        {
            if(i < motor_num / 2)
            {
                rb.Motors[i]->pos_vel_MAXtqe(angle, 0.3, 10);  
            }
            else
            {
                rb.Motors[i]->pos_vel_MAXtqe(0, 0.3, 10);  
            }
        }
        rb.motor_send_cmd();

        ++cont;
        if(cont >= 200)
        {
            cont = 0;
            angle *= -1;
        }
        
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