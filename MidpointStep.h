#pragma once

#include "Integrator.h"

#include <string>

class MidpointStep : public Integrator
{
public:
	MidpointStep();
	virtual ~MidpointStep();

	void integrate(RigidBody *rb, double timeStep);
	std::string getString();
private:
	/* data */
};
