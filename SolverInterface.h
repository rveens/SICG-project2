#pragma once

#include "Eigen/Dense"

using namespace Eigen;

class SolverInterface {
public:
	virtual VectorXd getState() = 0;
	virtual void setState(const VectorXd &state) = 0;
	virtual VectorXd derivEval() = 0;
	virtual VectorXd derivEval(const VectorXd &input) = 0;
};