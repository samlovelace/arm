#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "IManipComms.h"
#include <memory> 

class Manipulator
{
public:
    Manipulator(IManipComms* aManipComms);
    ~Manipulator();
private:
    std::unique_ptr<IManipComms> mManipComms;

};
#endif //MANIPULATOR_H
