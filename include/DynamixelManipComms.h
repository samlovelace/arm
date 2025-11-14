#ifndef DYNAMIXELMANIPCOMMS_H
#define DYNAMIXELMANIPCOMMS_H

#include <thread> 
#include <mutex> 
#include <memory> 
#include "IManipComms.h"
#include <yaml-cpp/yaml.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_PRESENT_SPEED           38
#define ADDR_MX_MAX_TORQUE              14

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
 

struct MotorConfig
{
    int mId; 
    double mMaxTorque; 
    double mStepConversion; 

    MotorConfig(int anId, double aMaxTorque, double aStepConversion) :
        mId(anId), mMaxTorque(aMaxTorque), mStepConversion(aStepConversion) {}
};

class DynamixelManipComms : public IManipComms
{ 
public:
    DynamixelManipComms(const YAML::Node& aCommsConfig);
    DynamixelManipComms() {}
    ~DynamixelManipComms() override;
   
    
   
    bool init() override; 
    KDL::JntArray getJointPositions() override; 
    KDL::JntArray getJointVelocities() override; 
    void sendJointCommand(const KDL::JntArray &aCmd) override; 

private: 

    void writeMaxTorque(const MotorConfig& aMotor);
    void commsLoop(); 
    void setLatestJointCommand(const KDL::JntArray& aCmd); 
    KDL::JntArray getLatestJointCommand(); 

    void setLatestJointPositions(const KDL::JntArray& aJntPos); 
    void setLatestJointVelocities(const KDL::JntArray& aJntVel);


    void toggleEnableState(const MotorConfig& aMotor, bool aState);
    void writeJointCommandToSerialPort(const KDL::JntArray& aJntCmd);
    void readStateFromSerialPort(KDL::JntArray& aJointPositions, KDL::JntArray& aJointVelocities);

private:

    dynamixel::PortHandler* mPortHandler; 
    dynamixel::PacketHandler* mPacketHandler; 
    std::unique_ptr<dynamixel::GroupSyncWrite> mJointPosWriter; 
    std::unique_ptr<dynamixel::GroupBulkRead> mJointStateReader; 

    std::string mDeviceName; 
    int mProtocolVersion; 
    int mBaudRate; 
    std::vector<MotorConfig> mMotors; 
   
    std::thread mCommsThread; 
    std::atomic<bool> mRunning;
    
    // command stuff 
    std::mutex mCmdMutex;
    KDL::JntArray mJointCommand;

    // state stuff 
    std::mutex mStateMutex; 
    KDL::JntArray mJointPositions; 
    KDL::JntArray mPrevJointPositions; 
    KDL::JntArray mJointVelocities; 

    std::chrono::steady_clock::time_point mPrevTime; 


};
#endif //DYNAMIXELMANIPCOMMS_H