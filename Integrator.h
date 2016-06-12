#pragma once

#include "RigidBody.h"

#include <string>

class Integrator
{
public:
	virtual void integrate(RigidBody *rb, double timeStep) = 0;
	virtual std::string getString() = 0;
};
