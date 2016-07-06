#ifndef SOLVER_H
#define SOLVER_H

#include <vector>

#include "RigidBody.h"
#include "Particle.h"
#include "Integrator.h"
#include "Force.h"
#include "CollisionSolver.h"

class Solver
{
public:
	Solver(float _dtfluid, float _dtrb, float _diff, float _visc, float _vort);
	virtual ~Solver();

	/* public functions: */
	void dens_step(int N, float * x, float * x0, float * u, float * v, int * solid);
	void vel_step(int N, float * u, float * v, float * u0, float * v0, int * solid);

	void rigidbodySolve(int N, float * u, float * v, int *solid, float *dens, float * p, float * div);
	void drawObjects(int N, int *solid);
	void addRigidBody(RigidBody *rb);
	void addParticle(Particle *p);
	void addForce(Force *f);
	void setIntegrator(Integrator *i);
	RigidBody *getRigidBodyOnMousePosition(double x, double y);

	/* drawing variables: */
	bool m_Drawbb = false;
	bool m_DrawbbCells = false;
	bool m_DrawbbCellsOccupied = false;
	bool m_DrawBoundaries = false;
	bool m_DrawEdgeNormals = false;
	bool m_DrawContacts = false;

private:
	Integrator *m_Integrator;
	CollisionSolver colsolver;
	
	/* private variables: */
	float dt;
	float dtrb;
	float diff;
	float visc;
	float vort;
	std::vector<RigidBody *> m_rbodies;
	std::vector<Particle *> m_particles;
	std::vector<Force *> m_forces;

	/* private functions: */ 

	// stuff for fluids
	void add_source(int N, float * x, float * s);
	void advect(int N, int b, float * d, float * d0, float * u, float * v, int * solid);
	void diffuse(int N, int b, float * x, float * x0, float diffvisc, int * solid);
	void lin_solve(int N, int b, float * x, float * x0, float a, float c, int * solid);
	void project(int N, float * u, float * v, float * p, float * div, int * solid);
	void confine_vorticity(int N, float * u, float * v, int * solid);
	double count_density(int N, float * x, int * solid);
	void preserve_density(int N, float * x, int * solid, double old_density, double new_density);

	// stuff for rigid bodies
	// drawing that doesnt really belong in rigid bodies
	void drawContactPoints();
};

#endif /* SOLVER_H */
