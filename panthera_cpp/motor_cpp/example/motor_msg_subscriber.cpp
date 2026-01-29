#include <lcm/lcm-cpp.hpp>
#include "motor_msg/motor_msg.hpp"  // Include the generated header for motor_msg
#include <iostream>

// Callback function to handle incoming LCM messages
class MotorMsgHandler
{
public:
    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const motor_msg::motor_msg* msg)
    {
        // Process the received motor_msg data
        printf("Received motor_msg:\n");
        printf("can1_num: %d\n", msg->can1_num);
        printf("can2_num: %d\n", msg->can2_num);
        printf("can3_num: %d\n", msg->can3_num);
        printf("can4_num: %d\n", msg->can4_num);
        printf("motor_msg: ");
        for (int i = 0; i < 40; ++i)
        {
            printf("%f ", msg->motor_status[i]);
        }
        printf("\n");
    }
};

int main()
{
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }

    MotorMsgHandler handler;
    lcm.subscribe("motor_msg", &MotorMsgHandler::handleMessage, &handler);

    printf("Subscribing to motor_msg. Press Ctrl-C to exit.\n");
    while (true)
    {
        lcm.handle();
    }

    return 0;
}