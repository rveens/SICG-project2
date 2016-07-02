#pragma once

#include "Eigen/Dense"

using namespace Eigen;

class Particle
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
	unsigned int getDim();		// nieuw
	void setState(const Vector4d &state);	// nieuw
	Vector4d getState();		// nieuw
	Vector4d derivEval(double timeStep);// nieuw
	Vector4d derivEval(const Vector4d &input); 	// nieuw
	Particle *checkSelected(double x_given, double y_given);

	Vector2d m_ConstructPos; 	// initial position?
	Vector2d m_Position;
	Vector2d m_Velocity;
	Vector2d m_Force;
	int m_Mass;
	bool m_Static = false;
private:
	const double h = 0.03;
	const double s = 0.03;
};
