#ifndef SOLVER_H
#define SOLVER_H

class Solver
{
public:
	Solver (float _dt, float _diff, float _visc);
	virtual ~Solver ();

	/* public functions: */
	void dens_step(int N, float * x, float * x0, float * u, float * v, int * solid);
	void vel_step(int N, float * u, float * v, float * u0, float * v0, int * solid);

	void rigidbodySolve();

private:
	/* private variables: */
	float dt;
	float diff;
	float visc;

	/* private functions: */ 

	// stuff for fluids
	void add_source(int N, float * x, float * s);
	void advect(int N, int b, float * d, float * d0, float * u, float * v, int * solid);
	void diffuse(int N, int b, float * x, float * x0, int * solid);
	void lin_solve(int N, int b, float * x, float * x0, float a, float c, int * solid);
	void project(int N, float * u, float * v, float * p, float * div, int * solid);
	void set_bnd(int N, int b, float * x, int * solid);
	void confine_vorticity(int N, float * u, float * v, int * solid);
};

#endif /* SOLVER_H */
