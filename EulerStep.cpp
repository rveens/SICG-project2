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

	std::cout << result << std::endl;
	rb->setState(result);

	std::cout << "velocity: " << rb->m_Velocity << std::endl;
}

std::string EulerStep::getString()
{
	return std::string("Euler");
}
