#include "robot.hpp"
#include <iostream>


int main(int argc, char **argv)
{
    hightorque_robot::robot rb;
    rb.lcm_enable();

    rb.canboard_bootloader();  // 进入烧录模式
}

