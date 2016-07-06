#include "GravityForce.h"
#include "RigidBody.h"
#include "Particle.h"

#include "./Eigen/Dense"

GravityForce::GravityForce(SolverInterface *obj) : m_obj(obj)
{

}

GravityForce::~GravityForce()
{

}

void GravityForce::calculateForce()
{
	// test:
	RigidBody *rb = dynamic_cast<RigidBody*> (m_obj);
	if (rb != nullptr) {
		rb->m_Force[1] += -rb->m_Mass * m_g;
	}
	Particle *p = dynamic_cast<Particle*> (m_obj);
	if (p != nullptr) {
		p->m_Force[1] += -p->m_Mass * m_g;
	}
}

void GravityForce::draw()
{
	// TODO
}
