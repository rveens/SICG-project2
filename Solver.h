#ifndef SOLVER_H
#define SOLVER_H

class Solver
{
public:
	Solver (float _dt, float _diff, float _visc);
	virtual ~Solver ();

	/* public functions: */
	void dens_step(int N, float * x, float * x0, float * u, float * v);
	void vel_step(int N, float * u, float * v, float * u0, float * v0);

	void rigidbodySolve();

private:
	/* private variables: */
	float dt;
	float diff;
	float visc;

	/* private functions: */ 

	// stuff for fluids
	void add_source(int N, float * x, float * s);
	void advect(int N, int b, float * d, float * d0, float * u, float * v);
	void diffuse(int N, int b, float * x, float * x0);
	void lin_solve(int N, int b, float * x, float * x0, float a, float c);
	void project(int N, float * u, float * v, float * p, float * div);
	void set_bnd(int N, int b, float * x);

	// stuff for rigid bodies
};

#endif /* SOLVER_H */
