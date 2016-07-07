#pragma once

#include "SolverInterface.h"

#include <string>
#include <memory>

class Integrator
{
public:
	virtual void integrate(std::shared_ptr<SolverInterface> obj, double timeStep) = 0;
	virtual std::string getString() = 0;
	virtual ~Integrator() { }
};
