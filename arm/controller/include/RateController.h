#ifndef RATECONTROLLER_H
#define RATECONTROLLER_H

#include <chrono> 
 
class RateController 
{ 
public:
    RateController(int aLoopRate);
    ~RateController();

    void start(); 
    void block(); 

private:

    std::chrono::steady_clock::time_point mStart;
    std::chrono::duration<double> mLoopDuration; 

   
};
#endif //RATECONTROLLER_H