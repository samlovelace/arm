#ifndef MANIPULATORFACTORY_H
#define MANIPULATORFACTORY_H

#include <memory>
#include <yaml-cpp/yaml.h>

#include "arm_hardware/IManipComms.hpp"

class ManipulatorFactory
{
public:
    ManipulatorFactory();
    ~ManipulatorFactory();

    static std::shared_ptr<IManipComms> create();
private:

};
#endif
