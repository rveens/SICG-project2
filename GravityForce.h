#pragma once

#include "Force.h"
#include "SolverInterface.h"


class GravityForce : public Force
{
public:
	GravityForce(SolverInterface *obj);
	virtual ~GravityForce();

	void calculateForce();
	void draw();
private:
	const double m_g = 9.81;
	SolverInterface *const m_obj;
};
