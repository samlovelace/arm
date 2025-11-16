#ifndef LOGGER_H
#define LOGGER_H

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

#ifdef _WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pwd.h>
#endif

namespace fs = std::filesystem;

// Function to check if a directory exists (cross-platform)
bool directoryExists(const std::string& directoryName) {
    return fs::exists(directoryName) && fs::is_directory(directoryName);
}

// Function to create directory with the specified name if it doesn't already exist (cross-platform)
void createDirectory(const std::string& directoryName) {
    if (!directoryExists(directoryName)) {
        fs::create_directories(directoryName);  // Create directories recursively if needed
    }
}

// Function to initialize logging
void createLogger() {
    // Get current date and time
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()) % 60;

    struct passwd *pw = getpwuid(getuid());
    if (pw == nullptr) {
        std::cerr << "Failed to get username.\n";
        return;
    }
    std::string username = pw->pw_name;

    // Directory path
    std::string dirPath = "/home/" + username + "/testing/";

    // Create "testing" directory if it doesn't already exist
    createDirectory(dirPath);

    // Format date in YYYY_MM_DD
    std::ostringstream dateSS;
    dateSS << dirPath + "/test_" << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d");
    std::string directoryName = dateSS.str();

    // Create directory if it doesn't already exist
    createDirectory(directoryName);

    // Format time in HH_MM_SS
    std::ostringstream timeSS;
    timeSS << std::put_time(std::localtime(&now_time_t), "%H_%M_") << std::setw(2) << std::setfill('0') << seconds.count();
    std::string logFileName = "arm_controller_log" + timeSS.str() + ".csv";
    //std::string datalogFileName = "NORA_DataLog_" + timeSS.str() + ".csv";

    // Full paths for log files
    std::string logFilePath = directoryName + "/" + logFileName;
    //std::string datalogFilePath = directoryName + "/" + datalogFileName;

    // Initialize logger with CSV file name and console logger
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender(logFilePath.c_str());  // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;  // Create the 2nd appender.
    plog::init(plog::verbose, &fileAppender).addAppender(&consoleAppender);  // Initialize the logger with both appenders.

    // Initialize a separate logger for the data log file
    //static plog::RollingFileAppender<plog::CsvFormatter> dataFileAppender(datalogFilePath.c_str());
    //plog::init<1>(plog::verbose, &dataFileAppender);  // Use a separate instance for the data log

    // Log the creation of the data log file
    //PLOG_INFO << datalogFileName << " created";
}

// Signal handler function
void signalHandler(int signal) {

    LOGD  << "\n" << "\t\t"
	  R"(_________________________
		|                       |
		|   SHUTTING DOWN...    |
		|_______________________|
               __   /
              / o) /
     _.----._/ /
    /         /
 __/ (  | (  |
/__.-'|_|--|_|
)";
    exit(0); // Exit the program
}

#endif // LOGGER_H