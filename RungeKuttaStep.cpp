#include "RungeKuttaStep.h"

#include <iostream>

RungeKuttaStep::RungeKuttaStep()
{

}

RungeKuttaStep::~RungeKuttaStep()
{

}

void RungeKuttaStep::integrate(std::shared_ptr<SolverInterface> obj, double timeStep)
{
	VectorXd k1 = timeStep * obj->derivEval();
	/* std::cout << "k1:" << std::endl; */
	/* std::cout << k1 << std::endl; */
	VectorXd k2 = timeStep * obj->derivEval(obj->getState() + k1/2.0);
	/* std::cout << "k2:" << std::endl; */
	/* std::cout << k2 << std::endl; */
	VectorXd k3 = timeStep * obj->derivEval(obj->getState() + k2/2.0);
	/* std::cout << "k3:" << std::endl; */
	/* std::cout << k3 << std::endl; */
	VectorXd k4 = timeStep * obj->derivEval(obj->getState() + k3);
	/* std::cout << "k4:" << std::endl; */
	/* std::cout << k4 << std::endl; */
	VectorXd k5 = obj->getState() + k1/6.0 + k2/3.0 + k3/3.0 + k4/6.0;
	/* std::cout << "k5:" << std::endl; */
	/* std::cout << k5 << std::endl; */

	obj->setState(k5);
}

std::string RungeKuttaStep::getString()
{
	return std::string("Runge-Kutta");
}
