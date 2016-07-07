#include "GravityForce.h"
#include "RigidBody.h"
#include "Particle.h"

#include "./Eigen/Dense"

GravityForce::GravityForce(std::shared_ptr<SolverInterface> obj) : m_obj(obj)
{

}

GravityForce::~GravityForce()
{

}

void GravityForce::calculateForce()
{
	// test:
	std::shared_ptr<RigidBody> rb = std::dynamic_pointer_cast<RigidBody>(m_obj);
	if (rb != nullptr) {
		rb->m_Force[1] += -rb->m_Mass * m_g;
	}
	std::shared_ptr<Particle> p = std::dynamic_pointer_cast<Particle>(m_obj);
	if (p != nullptr) {
		p->m_Force[1] += -p->m_Mass * m_g;
	}
}

void GravityForce::draw()
{
	// TODO
}
