#include "MidpointStep.h"

MidpointStep::MidpointStep()
{

}

MidpointStep::~MidpointStep()
{

}

void MidpointStep::integrate(std::shared_ptr<SolverInterface> obj, double timeStep)
{
	VectorXd k1 = timeStep * obj->derivEval();
	VectorXd k2 = timeStep * obj->derivEval(obj->getState() + k1/2.0);

	VectorXd k5 = obj->getState() + k2;

	obj->setState(k5);
}

std::string MidpointStep::getString()
{
	return std::string("Midpoint");
}
