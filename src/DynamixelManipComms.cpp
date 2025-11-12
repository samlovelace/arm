
#include "DynamixelManipComms.h"
#include "plog/Log.h"

DynamixelManipComms::DynamixelManipComms(const YAML::Node& aCommsConfig) : mPortHandler(nullptr)
{
    mDeviceName = aCommsConfig["device_name"].as<std::string>(); 
    mProtocolVersion = aCommsConfig["protocol_version"].as<int>(); 
    mBaudRate = aCommsConfig["baud_rate"].as<int>(); 

    std::vector<int> ids = aCommsConfig["motor_ids"].as<std::vector<int>>();
    std::vector<double> maxTorques = aCommsConfig["max_torque"].as<std::vector<double>>(); 
    std::vector<double> stepConversion = aCommsConfig["step_conversion"].as<std::vector<double>>(); 

    for(int i = 0; i < ids.size(); i++)
    {
        MotorConfig m(ids[i], maxTorques[i], stepConversion[i]);
        mMotors.push_back(m); 
    }
}

DynamixelManipComms::~DynamixelManipComms()
{

}

bool DynamixelManipComms::init()
{
    mPortHandler = dynamixel::PortHandler::getPortHandler(mDeviceName.c_str()); 
    mPacketHandler = dynamixel::PacketHandler::getPacketHandler(mProtocolVersion);
 
    if(nullptr == mPortHandler) throw std::runtime_error("Failed to instantiate PortHandler for device"); 

    if(!mPortHandler->openPort()) throw std::runtime_error("Failed to open port to dynamixel manipulator");  
    if(!mPortHandler->setBaudRate(mBaudRate)) throw std::runtime_error("Failed to set Baud Rate"); 

    // send configured settings to motors 
    for(const auto& motor : mMotors)
    {
        writeMaxTorque(motor);
    }
    
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL; 

    // Enable Dynamixel#1 Torque
    dxl_comm_result = mPacketHandler->write1ByteTxRx(mPortHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", mPacketHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", mPacketHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    return true; 
}

KDL::JntArray DynamixelManipComms::getJointPositions()
{
    // Read present position
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL; 
    uint16_t dxl_present_position = 0; 

    dxl_comm_result = mPacketHandler->read2ByteTxRx(mPortHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
    printf("%s\n", mPacketHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
    printf("%s\n", mPacketHandler->getRxPacketError(dxl_error));
    }

    KDL::JntArray pos(1); 
    pos(0) = dxl_present_position * 0.088; // convert to degrees 
    
    return pos; 
} 

KDL::JntArray DynamixelManipComms::getJointVelocities()
{

} 

void DynamixelManipComms::sendJointCommand(const KDL::JntArray &aCmd)
{
    double goal_pos = aCmd(0) / 0.088; 

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL; 

    // Write goal position
    dxl_comm_result = mPacketHandler->write2ByteTxRx(mPortHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, goal_pos, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", mPacketHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", mPacketHandler->getRxPacketError(dxl_error));
    }
} 

void DynamixelManipComms::writeMaxTorque(const MotorConfig& aMotor)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Disable torque before writing to EEPROM
    dxl_comm_result = mPacketHandler->write1ByteTxRx(mPortHandler, aMotor.mId, 
                                                     ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, 
                                                     &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        mPacketHandler->getTxRxResult(dxl_comm_result);
    if (dxl_error != 0)
        mPacketHandler->getRxPacketError(dxl_error);

    // Write max torque value
    uint16_t valToWrite = static_cast<uint16_t>(1023 * aMotor.mMaxTorque);
    LOGV << "Writing MaxTorque: " << valToWrite << " for Motor: " << aMotor.mId; 
    dxl_comm_result = mPacketHandler->write2ByteTxRx(mPortHandler, aMotor.mId, 
                                                     ADDR_MX_MAX_TORQUE, valToWrite, 
                                                     &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        mPacketHandler->getTxRxResult(dxl_comm_result);
    if (dxl_error != 0)
        mPacketHandler->getRxPacketError(dxl_error);
}
