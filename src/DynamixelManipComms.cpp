
#include "DynamixelManipComms.h"
#include "plog/Log.h"
#include "RateController.h"

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
    mRunning.store(false); 
    if(mCommsThread.joinable())
    {
        mCommsThread.join(); 
    }

    mPortHandler->closePort();
}

bool DynamixelManipComms::init()
{
    mPortHandler = dynamixel::PortHandler::getPortHandler(mDeviceName.c_str()); 
    mPacketHandler = dynamixel::PacketHandler::getPacketHandler(mProtocolVersion);
 
    if(nullptr == mPortHandler) throw std::runtime_error("Failed to instantiate PortHandler for device"); 

    if(!mPortHandler->openPort()) throw std::runtime_error("Failed to open port to dynamixel manipulator");  
    if(!mPortHandler->setBaudRate(mBaudRate)) throw std::runtime_error("Failed to set Baud Rate"); 

    // Initialize GroupSyncWrite instance for writing goal joint pos
    mJointPosWriter = std::make_unique<dynamixel::GroupSyncWrite>(mPortHandler, mPacketHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION); 
    if(nullptr == mJointPosWriter) throw std::runtime_error("Failed to initialize Goal Position Group Writer"); 

    // Initialize GroupBulkRead instance
    mJointStateReader = std::make_unique<dynamixel::GroupBulkRead>(mPortHandler, mPacketHandler);
    if(nullptr == mJointStateReader) throw std::runtime_error("Failed to initialize Group Reader"); 

    // setup joint state reader packet group
    for(int i = 0; i < mMotors.size(); i++)
    {
        mJointStateReader->addParam(mMotors[i].mId, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION); 
        mJointStateReader->addParam(mMotors[i].mId, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_POSITION);
    }

    // send configured settings to motors 
    for(const auto& motor : mMotors)
    {
        toggleEnableState(motor, false); // disable motor before writing new settings to it  
        
        // new config settings 
        writeMaxTorque(motor);

        toggleEnableState(motor, true); // re-enable motor 
    }

    mPrevJointPositions.resize(mMotors.size()); 
    mPrevTime = std::chrono::steady_clock::now(); 

    mCommsThread = std::thread(&DynamixelManipComms::commsLoop, this); 

    return true; 
}

void DynamixelManipComms::commsLoop()
{
    RateController rate(10);
    mRunning.store(true); 

    while(mRunning)
    {
        rate.start(); 

        // send latest joint command
        KDL::JntArray cmd = getLatestJointCommand(); 
        writeJointCommandToSerialPort(cmd); 

        // // get latest joint positions/velocities 
        KDL::JntArray pos, vel; 
        readStateFromSerialPort(pos, vel); 
        setLatestJointPositions(pos); 
        setLatestJointVelocities(vel);  
        
        rate.block(); 
    }
}

void DynamixelManipComms::writeJointCommandToSerialPort(const KDL::JntArray& aJntCmd)
{
    int cmdSize = aJntCmd.rows(); 
    int numMotors = mMotors.size(); 

    if(cmdSize > numMotors)
    {
        LOGW << "Attempted to command " << cmdSize << " joints for "
             << numMotors << "connected motors";  
    }
    else if (cmdSize < numMotors)
    {
        LOGE << "Attempted to command less joints (" << cmdSize << ")" 
             << " than connected motors (" << numMotors << ")."; 
        return;  // TODO: proper behavior for this? pad cmd to zeros to meet mMotors.size? 
    }

    // accumulate packet to send 
    for(int i = 0; i < mMotors.size(); i++)
    {
        double jntCmd = aJntCmd(i) / mMotors[i].mStepConversion; 
        uint16_t stepCmd = static_cast<uint16_t>(std::round(jntCmd));

        uint8_t goal[2]; 
        goal[0] = DXL_LOBYTE(stepCmd); 
        goal[1] = DXL_HIBYTE(stepCmd); 

        // add motor id and goal pos to group writer 
        mJointPosWriter->addParam(mMotors[i].mId, goal); 
    }

    // send packet 
    int result = mJointPosWriter->txPacket(); 
    if (result != COMM_SUCCESS) LOGW << mPacketHandler->getTxRxResult(result);

    // clear packet 
    mJointPosWriter->clearParam(); 
}

void DynamixelManipComms::readStateFromSerialPort(KDL::JntArray& aJointPositions, KDL::JntArray& aJointVelocities)
{
    // send request to read state(s)
    int result = mJointStateReader->txRxPacket(); 
    
    if (result != COMM_SUCCESS) 
    {
        LOGW << mPacketHandler->getTxRxResult(result);
        // TODO: handle error 
    }

    aJointPositions.resize(mMotors.size()); 
    aJointVelocities.resize(mMotors.size()); 

    auto nowTime = std::chrono::steady_clock::now(); 
    auto dt = std::chrono::duration<double>(nowTime - mPrevTime).count(); 

    for (int i = 0; i < mMotors.size(); i++)
    {
        bool positionRead = false; 

        if(mJointStateReader->isAvailable(mMotors[i].mId, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION))
        {
            uint32_t pos_steps = mJointStateReader->getData(mMotors[i].mId, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION); 
            double pos_deg = pos_steps * mMotors[i].mStepConversion; 
            aJointPositions(i) = pos_deg;
            positionRead = true;  
        }
        else
        {
            LOGW << "Data not available for motor: " << mMotors[i].mId << " position"; 
        }

        if(positionRead)
        {
            aJointVelocities(i) = (aJointPositions(i) - mPrevJointPositions(i)) / dt;
        }
    
        // if(mJointStateReader->isAvailable(mMotors[i].mId, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_POSITION))
        // {
        //     uint32_t vel_steps = mJointStateReader->getData(mMotors[i].mId, ADDR_MX_PRESENT_SPEED, LEN_MX_PRESENT_POSITION);
        //     double vel_rpm = vel_steps * 0.111; // convert to rpm via magic number from datasheet 

        //     if (1024 <= vel_steps <= 2048)
        //     {
        //         // motor turning CW, negative vel
        //         vel_rpm *= -1;  
        //     }

        //     aJointVelocities(i) = vel_rpm * (360.0 / 60.0); // convert to deg/s  
        // }
        // else
        // {
        //     LOGW << "Data not available for motor: " << mMotors[i].mId << " velocity"; 
        // }
    }

    mPrevJointPositions = aJointPositions; 
    mPrevTime = nowTime; 
}

KDL::JntArray DynamixelManipComms::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mJointPositions; 
} 

KDL::JntArray DynamixelManipComms::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mJointVelocities; 
} 

void DynamixelManipComms::sendJointCommand(const KDL::JntArray &aCmd)
{
    setLatestJointCommand(aCmd); 
} 

void DynamixelManipComms::setLatestJointCommand(const KDL::JntArray& aCmd)
{
    std::lock_guard<std::mutex> lock(mCmdMutex); 
    mJointCommand = aCmd; 
}

KDL::JntArray DynamixelManipComms::getLatestJointCommand()
{
    std::lock_guard<std::mutex> lock(mCmdMutex); 
    return mJointCommand; 
}

void DynamixelManipComms::setLatestJointPositions(const KDL::JntArray& aJntPos)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mJointPositions = aJntPos; 
}

void DynamixelManipComms::setLatestJointVelocities(const KDL::JntArray& aJntVel)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mJointVelocities = aJntVel; 
}

void DynamixelManipComms::toggleEnableState(const MotorConfig& aMotor, bool aState)
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

}

void DynamixelManipComms::writeMaxTorque(const MotorConfig& aMotor)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

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
