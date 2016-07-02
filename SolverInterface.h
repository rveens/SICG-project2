#pragma once

#include "Eigen/Dense"

using namespace Eigen;

class SolverInterface {
public:
	virtual VectorXd getState() = 0;
	virtual void setState(VectorXd state) = 0;
	virtual VectorXd derivEval() = 0;
	virtual VectorXd derivEval(VectorXd input) = 0;
	
	Vector2d m_Position;
	Vector2d m_Velocity;
	Vector2d m_Force;
	int m_Mass;
};