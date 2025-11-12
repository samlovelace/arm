#include <gtest/gtest.h>
#include "DynamixelManipComms.h"
#include <kdl/jntarray.hpp>

class DynamixelManipComms_Test : public ::testing::Test
{
public:
    DynamixelManipComms_Test(/* args */) {}
    ~DynamixelManipComms_Test() {}

    void SetUp() override
    {
        instance = std::make_shared<DynamixelManipComms>(); 
    }

    void TearDown() override
    {

    }

protected: 
    std::shared_ptr<IManipComms> instance;
};

TEST_F(DynamixelManipComms_Test, DynamixelCommsTest)
{
    ASSERT_TRUE(instance->init()); 
    sleep(2); 

    KDL::JntArray first(1);
    first(0) = 90; 

    KDL::JntArray second(1); 
    second(0) = 180; 

    instance->sendJointCommand(first); 
    
    for(int i = 0; i < 10; i++)
    {
        KDL::JntArray pos = instance->getJointPositions(); 
        std::cout << pos(0) << std::endl; 
    }

    sleep(3);
    instance->sendJointCommand(second);
    for(int i = 0; i < 10; i++)
    {
        KDL::JntArray pos = instance->getJointPositions(); 
        std::cout << pos(0) << std::endl; 
    }

    sleep(3); 
    instance->sendJointCommand(first);
    for(int i = 0; i < 10; i++)
    {
        KDL::JntArray pos = instance->getJointPositions(); 
        std::cout << pos(0) << std::endl; 
    } 
    sleep(3); 
}