#ifndef SOLVER_H
#define SOLVER_H

#include <vector>

#include "RigidBody.h"
#include "Integrator.h"
#include "Force.h"
#include "CollisionSolver.h"

class Solver
{
public:
	Solver(float _dtfluid, float _dtrb, float _diff, float _visc);
	virtual ~Solver();

	/* public functions: */
	void dens_step(int N, float * x, float * x0, float * u, float * v, int * solid);
	void vel_step(int N, float * u, float * v, float * u0, float * v0, int * solid);

	void rigidbodySolve(int N);
	void drawRigidBodies(int N);
	void addRigidBody(RigidBody *rb);
	void addForce(Force *f);
	void setIntegrator(Integrator *i);

private:
	Integrator *m_Integrator;
	CollisionSolver colsolver;
	
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
	void advect(int N, int b, float * d, float * d0, float * u, float * v, int * solid);
	void diffuse(int N, int b, float * x, float * x0, int * solid);
	void lin_solve(int N, int b, float * x, float * x0, float a, float c, int * solid);
	void project(int N, float * u, float * v, float * p, float * div, int * solid);
	void set_bnd(int N, int b, float * x, int * solid);
	void confine_vorticity(int N, float * u, float * v, int * solid);

	// stuff for rigid bodies
	void getPointOfCollision(double timeStep);
};

#endif /* SOLVER_H */
