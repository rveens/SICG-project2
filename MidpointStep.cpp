#include "MidpointStep.h"

MidpointStep::MidpointStep()
{

}

MidpointStep::~MidpointStep()
{

}

void MidpointStep::integrate(RigidBody *rb, double timeStep)
{
	VectorXd k1 = timeStep * rb->derivEval();
	VectorXd k2 = timeStep * rb->derivEval(rb->getState() + k1/2.0);

	VectorXd k5 = rb->getState() + k2;

	rb->setState(k5);
}

std::string MidpointStep::getString()
{
	return std::string("Midpoint");
}
