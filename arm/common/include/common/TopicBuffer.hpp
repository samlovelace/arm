#ifndef TOPICBUFFER_HPP
#define TOPICBUFFER_HPP

#include <mutex>

template<typename T>
struct TopicBuffer
{
    std::mutex mMtx;
    T mLastMsg;
    bool mHasMsg = false;
};

#endif