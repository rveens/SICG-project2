#pragma once

#include "Eigen/Dense"

#include "SolverInterface.h"

using namespace Eigen;

class Particle : public SolverInterface
{
public:
	enum SELECTION {
		OFF,
		GREEN,
		RED
	};

public:
	Particle(const Vector2d & ConstructPos, double mass);
	virtual ~Particle(void);

	void reset();
	void draw();
	virtual void setState(const VectorXd &state);	// nieuw
	VectorXd getState();	// nieuw
	VectorXd derivEval();	// nieuw
	VectorXd derivEval(const VectorXd &input); 	// nieuw
	Particle *checkSelected(double x_given, double y_given);

	Vector2d m_ConstructPos; 	// initial position?
	Vector2d m_Position;
	Vector2d m_Velocity;
	Vector2d m_Force;
	double m_Mass;
	bool m_Static = false;
private:
	const double h = 0.01; // TODO: make dependent of grid size
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
