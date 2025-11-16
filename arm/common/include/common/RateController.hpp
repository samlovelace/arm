#ifndef RATECONTROLLER_H
#define RATECONTROLLER_H

#include <chrono>
#include <thread>
#include <iostream>

class RateController 
{ 
public:
    RateController(int aLoopRate)
        : mLoopDuration(std::chrono::duration<double>(1.0 / aLoopRate)),
          mLastDeltaCheck(std::chrono::steady_clock::now()) {}

    ~RateController() = default;

    void start() {
        mStart = std::chrono::steady_clock::now();
    }

    void block() {
        auto end = std::chrono::steady_clock::now();
        auto elapsed = end - mStart;

        if (elapsed < mLoopDuration) {
            std::this_thread::sleep_for(mLoopDuration - elapsed);
        } else {
            std::cerr << "Loop overrun! Elapsed time: "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                      << " ms\n";
        }
    }

    double getDeltaTime() {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> delta = now - mLastDeltaCheck;
        mLastDeltaCheck = now;
        return delta.count(); // seconds
    }

private:
    std::chrono::steady_clock::time_point mStart;
    std::chrono::steady_clock::time_point mLastDeltaCheck;
    std::chrono::duration<double> mLoopDuration;
};

#endif // RATECONTROLLER_H