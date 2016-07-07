#include "EulerStep.h"

#include <vector>
#include <iostream>

EulerStep::EulerStep()
{

}

EulerStep::~EulerStep()
{

}

void EulerStep::integrate(std::shared_ptr<SolverInterface> obj, double timeStep)
{
	VectorXd F = obj->derivEval();
	VectorXd result = obj->getState() + timeStep*F;

	obj->setState(result);
}

std::string EulerStep::getString()
{
	return std::string("Euler");
}
