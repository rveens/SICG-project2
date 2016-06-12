#include "RungeKuttaStep.h"

#include <iostream>

RungeKuttaStep::RungeKuttaStep()
{

}

RungeKuttaStep::~RungeKuttaStep()
{

}

void RungeKuttaStep::integrate(RigidBody *rb, double timeStep)
{
	VectorXd k1 = timeStep * rb->derivEval();
	/* std::cout << "k1:" << std::endl; */
	/* std::cout << k1 << std::endl; */
	VectorXd k2 = timeStep * rb->derivEval(rb->getState() + k1/2.0);
	/* std::cout << "k2:" << std::endl; */
	/* std::cout << k2 << std::endl; */
	VectorXd k3 = timeStep * rb->derivEval(rb->getState() + k2/2.0);
	/* std::cout << "k3:" << std::endl; */
	/* std::cout << k3 << std::endl; */
	VectorXd k4 = timeStep * rb->derivEval(rb->getState() + k3);
	/* std::cout << "k4:" << std::endl; */
	/* std::cout << k4 << std::endl; */
	VectorXd k5 = rb->getState() + k1/6.0 + k2/3.0 + k3/3.0 + k4/6.0;
	/* std::cout << "k5:" << std::endl; */
	/* std::cout << k5 << std::endl; */

	rb->setState(k5);
}

std::string RungeKuttaStep::getString()
{
	return std::string("Runge-Kutta");
}
