#ifndef SOLVER_H
#define SOLVER_H

#include <vector>

#include "RigidBody.h"
#include "Integrator.h"
#include "Force.h"

class Solver
{
public:
	Solver(float _dtfluid, float _dtrb, float _diff, float _visc);
	virtual ~Solver();

	/* public functions: */
	void dens_step(int N, float * x, float * x0, float * u, float * v);
	void vel_step(int N, float * u, float * v, float * u0, float * v0);

	void rigidbodySolve();
	void drawRigidBodies();
	void addRigidBody(RigidBody *rb);
	void addForce(Force *f);
	void setIntegrator(Integrator *i);

private:
	Integrator *m_Integrator;
	
	/* private variables: */
	float dt;
	float dtrb;
	float diff;
	float visc;
	std::vector<RigidBody *> m_rbodies;
	std::vector<Force *> m_forces;

	/* private functions: */ 

	// stuff for fluids
	void add_source(int N, float * x, float * s);
	void advect(int N, int b, float * d, float * d0, float * u, float * v);
	void diffuse(int N, int b, float * x, float * x0);
	void lin_solve(int N, int b, float * x, float * x0, float a, float c);
	void project(int N, float * u, float * v, float * p, float * div);
	void set_bnd(int N, int b, float * x);
	void confine_vorticity(int N, float * u, float * v);
};

#endif /* SOLVER_H */
