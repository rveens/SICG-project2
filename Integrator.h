#pragma once

#include "SolverInterface.h"

#include <string>

class Integrator
{
public:
	virtual void integrate(SolverInterface *obj, double timeStep) = 0;
	virtual std::string getString() = 0;
	virtual ~Integrator() { }
};
