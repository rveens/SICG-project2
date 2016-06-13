#include "GravityForce.h"

#include "./Eigen/Dense"

GravityForce::GravityForce(RigidBody *rb) : m_rb(rb)
{

}

GravityForce::~GravityForce()
{

}

void GravityForce::calculateForce()
{
	m_rb->m_Force[1] += - m_rb->m_Mass * m_g;
	/* m_rb->m_Torque[0] += -0.001; */
	/* m_rb->m_Torque[1] += -0.001; */
}

void GravityForce::draw()
{
	// TODO
}
