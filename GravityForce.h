#pragma once

#include "Force.h"
#include "SolverInterface.h"

#include <memory>

class GravityForce : public Force
{
public:
	GravityForce(std::shared_ptr<SolverInterface> obj);
	virtual ~GravityForce();

	void calculateForce();
	void draw();
private:
	const double m_g = 9.81;
	const std::shared_ptr<SolverInterface> m_obj;
};
