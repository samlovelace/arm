#ifndef MANIPULATORFACTORY_H
#define MANIPULATORFACTORY_H

#include "IManipComms.h"

class ManipulatorFactory
{
public:
    ManipulatorFactory(/* args */);
    ~ManipulatorFactory();

    static IManipComms* create(const std::string& aManipType); 
private:
    /* data */

};
#endif