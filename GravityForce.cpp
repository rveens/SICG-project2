#include "GravityForce.h"
#include "RigidBody.h"

#include "./Eigen/Dense"

GravityForce::GravityForce(SolverInterface *obj) : m_obj(obj)
{

}

GravityForce::~GravityForce()
{

}

void GravityForce::calculateForce()
{
	/* m_rb->m_Force[1] += - m_rb->m_Mass * m_g; */

	// test:
	RigidBody *rb = dynamic_cast<RigidBody*> (m_obj);
	if (rb != nullptr) {
		rb->m_Torque[0] = 0.4;
		rb->m_Torque[1] = 0.4;
	}
}

void GravityForce::draw()
{
	// TODO
}
