
#include "Manipulator.h"

Manipulator::Manipulator(IManipComms* aManipComms) : mManipComms(std::make_unique<IManipComms>(aManipComms))
{

}

Manipulator::~Manipulator()
{

}

