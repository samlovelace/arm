#ifndef RATECONTROLLER_H
#define RATECONTROLLER_H

#include <chrono>
#include <thread>

class RateController
{
public:
    RateController(int aLoopRate)
    {
        mLoopDuration = std::chrono::duration<double>(1.0 / aLoopRate);
    }

    ~RateController() = default;

    void start()
    {
        mStart = std::chrono::steady_clock::now();
    }

    void block()
    {
        // Calculate the time taken for the loop iteration
        auto end = std::chrono::steady_clock::now();
        auto elapsed = end - mStart;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < mLoopDuration) {
            std::this_thread::sleep_for(mLoopDuration - elapsed);
        } else {
            // std::cerr << "Loop overrun! Elapsed time: "
            //             << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
            //             << " ms\n";
                }
    }

private:
    std::chrono::steady_clock::time_point mStart;
    std::chrono::duration<double> mLoopDuration;
};
#endif //RATECONTROLLER_H
