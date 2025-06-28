#ifndef MANIPULATORFACTORY_H
#define MANIPULATORFACTORY_H

#include "IManipComms.h"
#include <memory> 

class ManipulatorFactory
{
public:
    ManipulatorFactory(/* args */);
    ~ManipulatorFactory();

    static std::shared_ptr<IManipComms> create(const std::string& aManipType, const std::string& aCommsType); 
private:
    /* data */

};
#endif