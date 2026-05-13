#ifndef LOGGER_H
#define LOGGER_H

#include <memory>
#include <atomic>
#include <string>
#include <fstream>
#include <unordered_map>
#include <mutex>

using LogId = uint32_t;
static constexpr LogId INVALID_LOG = 0;

class Logger
{
public:

    static Logger& get()
    {
        static Logger instance;
        return instance;
    }

    void createMainLog(const std::string& aMainLogKey);

    LogId createLog(const std::string& aLogFileName);
    void write(LogId, const std::string& aLineToWrite);
    void write(LogId aLogId, const std::vector<double>& aVectorOfData);

private:

    Logger() : mMainLogCreated(false) {}
    ~Logger() = default;

    bool directoryExists(const std::string& directoryName);
    void createDirectory(const std::string& directoryName);

private:
    bool mMainLogCreated;
    std::string mMainLogTimestamp;
    std::string mMainLogDir;

    struct Log
    {
        std::ofstream mFile;
    };

    std::atomic<LogId> mNextId{1}; // 0 is reserved for invalid
    std::unordered_map<LogId, std::unique_ptr<Log>> mLogs;
    std::mutex mMutex;
};

#endif //LOGGER_H
