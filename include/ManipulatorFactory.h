#ifndef MANIPULATORFACTORY_H
#define MANIPULATORFACTORY_H

#include "IManipComms.h"
#include <memory> 
#include <yaml-cpp/yaml.h>
class ManipulatorFactory
{
public:
    ManipulatorFactory(/* args */);
    ~ManipulatorFactory();

    static std::shared_ptr<IManipComms> create(const YAML::Node& aConfig); 
private:
    /* data */

};
#endif