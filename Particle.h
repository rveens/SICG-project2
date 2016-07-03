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

	Particle(const Vector2d & ConstructPos, int mass);
	virtual ~Particle(void);

	void reset();
	void draw();
	virtual void setState(const VectorXd &state);	// nieuw
	virtual VectorXd getState();		// nieuw
	virtual VectorXd derivEval();// nieuw
	virtual VectorXd derivEval(const VectorXd &input); 	// nieuw
	Particle *checkSelected(double x_given, double y_given);

	Vector2d m_ConstructPos; 	// initial position?
	Vector2d m_Position;
	Vector2d m_Velocity;
	Vector2d m_Force;
	int m_Mass;
	bool m_Static = false;
private:
	const double h = 0.001;
};
