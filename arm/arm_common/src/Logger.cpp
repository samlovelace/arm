

#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <string>
#include <sstream>
#include <filesystem>  // Cross-platform file system library
#include "plog/Log.h"
#include "plog/Init.h"
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Appenders/ColorConsoleAppender.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pwd.h>

#include "arm_common/Logger.h"

void Logger::createMainLog(const std::string& aMainLogKey)
{
    if(mMainLogCreated)
    {
        LOGW << "Main log already created";
        return;
    }

    // Get current date and time
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()) % 60;

    struct passwd *pw = getpwuid(getuid());
    if (pw == nullptr)
    {
        std::cerr << "Failed to get username.\n";
        return;
    }
    std::string username = pw->pw_name;

    // Directory path
    std::string dirPath = "/home/" + username + "/testing/";

    // Format date in YYYY_MM_DD
    std::ostringstream dateSS;
    dateSS << dirPath + "/test_" << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d");
    mMainLogDir = dateSS.str();

    // Create directory if it doesn't already exist
    createDirectory(mMainLogDir);

    // Format time in HH_MM_SS
    std::ostringstream timeSS;
    timeSS << std::put_time(std::localtime(&now_time_t), "%H_%M_") << std::setw(2) << std::setfill('0') << seconds.count();
    mMainLogTimestamp = timeSS.str();
    std::string logFileName = aMainLogKey + "_log_" + mMainLogTimestamp + ".csv";

    // Full paths for log files
    std::string logFilePath = mMainLogDir + "/" + logFileName;

    // Initialize logger with CSV file name and console logger
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender(logFilePath.c_str());  // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;  // Create the 2nd appender.
    plog::init(plog::verbose, &fileAppender).addAppender(&consoleAppender);  // Initialize the logger with both appenders.

    mMainLogCreated = true;
}

LogId Logger::createLog(const std::string& aFileNameKey)
{
    std::lock_guard<std::mutex> lock(mMutex);

    auto log = std::make_unique<Log>();

    std::string fileName = aFileNameKey + "_" + mMainLogTimestamp + ".csv";
    std::string fullFilePath = mMainLogDir + "/" + fileName;
    log->mFile.open(fullFilePath, std::ios::out | std::ios::app);

    if(!log->mFile.is_open())
    {
        // assumes main log has been created so im going to use plog
        LOGW << "Failed to create log file for " << aFileNameKey;
        return INVALID_LOG;
    }

    LogId id = mNextId++; // get the unique id for this log

    mLogs.emplace(id, std::move(log));
    return id;
}

void Logger::write(LogId aLogId, const std::string& aLineToWrite)
{
    std::lock_guard<std::mutex> lock(mMutex);

    auto it = mLogs.find(aLogId);
    if (it == mLogs.end())
    {
        LOGW << "Log file does not exist for id: " << aLogId;
        return;
    }

    it->second->mFile << aLineToWrite << '\n';
}

void Logger::write(LogId aLogId, const std::vector<double>& aVectorOfData)
{
    // convert the data to a string
    std::stringstream ss;
    for(const auto val : aVectorOfData)
    {
        ss << std::to_string(val) << ",";
    }

    write(aLogId, ss.str());
}

bool Logger::directoryExists(const std::string& directoryName)
{
    return std::filesystem::exists(directoryName) && std::filesystem::is_directory(directoryName);
}

void Logger::createDirectory(const std::string& directoryName)
{
    if (!directoryExists(directoryName))
    {
        std::filesystem::create_directories(directoryName);  // Create directories recursively if needed
    }
}
