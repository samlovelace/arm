#include <gtest/gtest.h>
#include "DynamixelManipComms.h"
#include <kdl/jntarray.hpp>
#include "RateController.h"

class DynamixelManipComms_Test : public ::testing::Test
{
public:
    DynamixelManipComms_Test(/* args */) {}
    ~DynamixelManipComms_Test() {}

    void SetUp() override
    {
        YAML::Node comms;
        comms["device_name"] = "/dev/ttyUSB0";
        comms["baud_rate"]  = 2000000;
        comms["protocol_version"]  = 1;
        comms["motor_ids"] = std::vector<int>{1, 2}; 
        comms["max_torque"] = std::vector<double>{0.35, 0.35}; 
        comms["step_conversion"] = std::vector<double>{0.088, 0.088}; 
        comms["comms_rate"] = 30;  

        instance = std::make_shared<DynamixelManipComms>(comms); 
    }

    void TearDown() override
    {

    }

protected: 
    std::shared_ptr<IManipComms> instance;
};

void sendLoop(std::shared_ptr<IManipComms> comms)
{
    KDL::JntArray first(2);
    first(0) = 90;
    first(1) = 90;  

    KDL::JntArray second(2); 
    second(0) = 180;
    second(1) = 180;  

    int num = 0; 
    int count = 100; 

    RateController rate(10); 

    while(num < count)
    {
        rate.start();

        first(0) -= 1; 
        first(1) -= 2; 
        comms->sendJointCommand(first); 

        rate.block(); 
        num++; 
    }

}

void readLoop(std::shared_ptr<IManipComms> comms)
{
    int num = 0; 
    int count = 100; 

    RateController rate(10);

    while(num < count)
    {
        rate.start();

        KDL::JntArray pos = comms->getJointPositions(); 
        KDL::JntArray vel = comms->getJointVelocities(); 

        std::cout << "Pos: " << pos.data << std::endl; 
        std::cout << "Vel: " << vel.data << std::endl; 

        rate.block(); 
        num++; 
    }
}

TEST_F(DynamixelManipComms_Test, WriteCommand)
{
    ASSERT_TRUE(instance->init()); 
    sleep(2); 

    std::thread t1(readLoop, instance);
    std::thread t2(sendLoop, instance);

    t1.join();
    t2.join();
}