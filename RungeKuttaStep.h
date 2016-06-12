#pragma once

#include "Integrator.h"

#include <string>

class RungeKuttaStep : public Integrator
{
public:
	RungeKuttaStep();
	virtual ~RungeKuttaStep();

	void integrate(RigidBody *rb, double timeStep);
	std::string getString();
private:
	/* data */
};
