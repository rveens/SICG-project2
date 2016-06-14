#include "EulerStep.h"

#include <vector>
#include <iostream>

EulerStep::EulerStep()
{

}

EulerStep::~EulerStep()
{

}

void EulerStep::integrate(RigidBody *rb, double timeStep)
{
	VectorXd F = rb->derivEval();
	VectorXd result = rb->getState() + timeStep*F;

	rb->setState(result);
}

std::string EulerStep::getString()
{
	return std::string("Euler");
}
