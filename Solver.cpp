#include "Solver.h"
#include "RungeKuttaStep.h"

#include <cmath>
#include <iostream>

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) { if (solid[((i)+(N+2)*(j))]==0) {
#define END_FOR }}}


Solver::Solver(float _dtfluid, float _dtrb, float _diff, float _visc) :
	m_Integrator(new RungeKuttaStep()), dt(_dtfluid), dtrb(_dtrb),
	diff(_diff), visc(_visc)
{

}

Solver::~Solver()
{
	delete m_Integrator;
}

/* private functions: */

void Solver::add_source ( int N, float * x, float * s)
{
	int i, size=(N+2)*(N+2);
	for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

/* old version */
//void Solver::set_bnd ( int N, int b, float * x )
//{
//	int i;
//
//	for ( i=1 ; i<=N ; i++ ) {
//		x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
//		x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
//		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
//		x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
//	}
//	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
//	x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
//	x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
//	x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
//}

/* test version with thick outside boundary and inner cube */
//void Solver::set_bnd ( int N, int b, float * x )
//{
//	int i;
//	int s=5; // boundary size/thickness - 1
//
//
//	for (i = s + 1; i <= N - s; i++) {
//		x[IX(s        , i        )] = b == 1 ? -x[IX(s + 1, i)] : x[IX(s + 1, i)];
//		x[IX(N + 1 - s, i        )] = b == 1 ? -x[IX(N - s, i)] : x[IX(N - s, i)];
//		x[IX(i        , s        )] = b == 2 ? -x[IX(i, s + 1)] : x[IX(i, s + 1)];
//		x[IX(i        , N + 1 - s)] = b == 2 ? -x[IX(i, N - s)] : x[IX(i, N - s)];
//	}
//	x[IX(s        , s        )] = 0.5f*(x[IX(s + 1, s)] + x[IX(s, s + 1)]);
//	x[IX(s        , N + 1 - s)] = 0.5f*(x[IX(s + 1, N + 1 - s)] + x[IX(s, N - s)]);
//	x[IX(N + 1 - s, s        )] = 0.5f*(x[IX(N - s, s)] + x[IX(N + 1 - s, s + 1)]);
//	x[IX(N + 1 - s, N + 1 - s)] = 0.5f*(x[IX(N - s, N + 1 - s)] + x[IX(N + 1 - s, N - s)]);
//
//	
//	s = 20; // inner cube
//	for (i = s + 1; i <= N - s; i++) {
//		x[IX(s        , i        )] = b == 1 ? -x[IX(s - 1, i)] : x[IX(s - 1, i)];
//		x[IX(N + 1 - s, i        )] = b == 1 ? -x[IX(N+2-s, i)] : x[IX(N+2-s, i)];
//		x[IX(i        , s        )] = b == 2 ? -x[IX(i, s - 1)] : x[IX(i, s - 1)];
//		x[IX(i        , N + 1 - s)] = b == 2 ? -x[IX(i, N+2-s)] : x[IX(i, N+2-s)];
//	}
//	x[IX(s        , s        )] = 0.5f*(x[IX(s + 1, s)] + x[IX(s, s + 1)]);
//	x[IX(s        , N + 1 - s)] = 0.5f*(x[IX(s + 1, N + 1 - s)] + x[IX(s, N - s)]);
//	x[IX(N + 1 - s, s        )] = 0.5f*(x[IX(N - s, s)] + x[IX(N + 1 - s, s + 1)]);
//	x[IX(N + 1 - s, N + 1 - s)] = 0.5f*(x[IX(N - s, N + 1 - s)] + x[IX(N + 1 - s, N - s)]);
//}

/* solver for viscuous fluids, with no-slip boundary condition */
//void Solver::set_bnd(int N, int b, float * x, int * solid)
//{
//	int i, j, size = (N + 2)*(N + 2);
//	if (b == 2) b = 1;
//
//	for (i = 0; i <= N + 1; i++) {
//		for (j = 0; j <= N + 1; j++){
//			switch (solid[IX(i, j)]) {
//			case 0: // no solid:
//				// no change
//				break;
//			case 1: // left top:
//				x[IX(i, j)] = b == 1 ? -0.5f*(x[IX(i - 1, j)] + x[IX(i, j + 1)]) : 0.5f*(x[IX(i - 1, j)] + x[IX(i, j + 1)]);
//				break;
//			case 2: // top:
//				x[IX(i, j)] = b == 1 ? -x[IX(i, j + 1)] : x[IX(i, j + 1)];
//				break;
//			case 3: // right top:
//				x[IX(i, j)] = b == 1 ? -0.5f*(x[IX(i + 1, j)] + x[IX(i, j + 1)]) : 0.5f*(x[IX(i + 1, j)] + x[IX(i, j + 1)]);
//				break;
//			case 4: // left:
//				x[IX(i, j)] = b == 1 ? -x[IX(i - 1, j)] : x[IX(i - 1, j)];
//				break;
//			case 5: // solid - no border:
//				x[IX(i, j)] = 0;
//				break;
//			case 6: // right:
//				x[IX(i, j)] = b == 1 ? -x[IX(i + 1, j)] : x[IX(i + 1, j)];
//				break;
//			case 7: // left bottom:
//				x[IX(i, j)] = b == 1 ? -0.5f*(x[IX(i - 1, j)] + x[IX(i, j - 1)]) : 0.5f*(x[IX(i - 1, j)] + x[IX(i, j - 1)]);
//				break;
//			case 8: // bottom:
//				x[IX(i, j)] = b == 1 ? -x[IX(i, j - 1)] : x[IX(i, j - 1)];
//				break;
//			case 9: // right bottom:
//				x[IX(i, j)] = b == 1 ? -0.5f*(x[IX(i + 1, j)] + x[IX(i, j - 1)]) : 0.5f*(x[IX(i + 1, j)] + x[IX(i, j - 1)]);
//				break;
//			case 10: // inner corner left top:
//				x[IX(i, j)] = 0.5f*(x[IX(i - 1, j)] + x[IX(i, j + 1)]);
//				break;
//			case 11: // inner corner right top:
//				x[IX(i, j)] = 0.5f*(x[IX(i + 1, j)] + x[IX(i, j + 1)]);
//				break;
//			case 12: // inner corner left bottom:
//				x[IX(i, j)] = 0.5f*(x[IX(i - 1, j)] + x[IX(i, j - 1)]);
//				break;
//			case 13: // inner corner right bottom:
//				x[IX(i, j)] = 0.5f*(x[IX(i + 1, j)] + x[IX(i, j - 1)]);
//				break;
//			}
//		}
//	}
//}

/* solver for normal (non-viscuous) fluids */
void Solver::set_bnd(int N, int b, float * x, int * solid)
{
	int i, j, size = (N + 2)*(N + 2);

	for (i = 0; i <= N + 1; i++) {
		for (j = 0; j <= N + 1; j++){
			switch (solid[IX(i, j)]) {
			case 0: // no solid:
				// no change
				break;
			case 1: // left top:
				x[IX(i, j)] = b == 1 ? -x[IX(i - 1, j)] : (b == 2 ? -x[IX(i, j + 1)] : 0.5f*(x[IX(i - 1, j)] + x[IX(i, j + 1)]));
				break;
			case 2: // top:
				x[IX(i, j)] = b == 2 ? -x[IX(i, j + 1)] : x[IX(i, j + 1)];
				break;
			case 3: // right top:
				x[IX(i, j)] = b == 1 ? -x[IX(i + 1, j)] : (b == 2 ? -x[IX(i, j + 1)] : 0.5f*(x[IX(i + 1, j)] + x[IX(i, j + 1)]));
				break;
			case 4: // left:
				x[IX(i, j)] = b == 1 ? -x[IX(i - 1, j)] : x[IX(i - 1, j)];
				break;
			case 5: // solid - no border:
				x[IX(i, j)] = 0;
				break;
			case 6: // right:
				x[IX(i, j)] = b == 1 ? -x[IX(i + 1, j)] : x[IX(i + 1, j)];
				break;
			case 7: // left bottom:
				x[IX(i, j)] = b == 1 ? -x[IX(i - 1, j)] : (b == 2 ? -x[IX(i, j - 1)] : 0.5f*(x[IX(i - 1, j)] + x[IX(i, j - 1)]));
				break;
			case 8: // bottom:
				x[IX(i, j)] = b == 2 ? -x[IX(i, j - 1)] : x[IX(i, j - 1)];
				break;
			case 9: // right bottom:
				x[IX(i, j)] = b == 1 ? -x[IX(i + 1, j)] : (b == 2 ? -x[IX(i, j - 1)] : 0.5f*(x[IX(i + 1, j)] + x[IX(i, j - 1)]));
				break;
			case 10: // inner corner left top:
				x[IX(i, j)] = 0.5f*(x[IX(i - 1, j)] + x[IX(i, j + 1)]);
				break;
			case 11: // inner corner right top:
				x[IX(i, j)] = 0.5f*(x[IX(i + 1, j)] + x[IX(i, j + 1)]);
				break;
			case 12: // inner corner left bottom:
				x[IX(i, j)] = 0.5f*(x[IX(i - 1, j)] + x[IX(i, j - 1)]);
				break;
			case 13: // inner corner right bottom:
				x[IX(i, j)] = 0.5f*(x[IX(i + 1, j)] + x[IX(i, j - 1)]);
				break;
			}
		}
	}
}


void Solver::lin_solve ( int N, int b, float * x, float * x0, float a, float c, int * solid )
{
	int i, j, k;

	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
		END_FOR
		set_bnd ( N, b, x, solid );
	}
}

void Solver::diffuse ( int N, int b, float * x, float * x0, int * solid)
{
	float a=dt*diff*N*N;
	lin_solve ( N, b, x, x0, a, 1+4*a, solid );
}

void Solver::advect ( int N, int b, float * d, float * d0, float * u, float * v, int * solid)
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*N;
	FOR_EACH_CELL
		x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
		if (x<0.5f) x=0.5f; if (x>N+0.5f) x=N+0.5f; i0=(int)x; i1=i0+1;
		if (y<0.5f) y=0.5f; if (y>N+0.5f) y=N+0.5f; j0=(int)y; j1=j0+1;
		s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
		d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
					 s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
	END_FOR
	set_bnd ( N, b, d, solid );
}

void Solver::project ( int N, float * u, float * v, float * p, float * div, int * solid )
{
	int i, j;

	FOR_EACH_CELL
		div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
		p[IX(i,j)] = 0;
	END_FOR	
	set_bnd ( N, 0, div, solid ); set_bnd ( N, 0, p, solid );

	lin_solve ( N, 0, p, div, 1, 4, solid );

	FOR_EACH_CELL
		u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
		v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
	END_FOR
	set_bnd ( N, 1, u, solid ); set_bnd ( N, 2, v, solid );
}

void Solver::confine_vorticity(int N, float * u, float * v, int * solid)
{
	// TODO: bring this out as a parameter!
	float eps = 1.0f;
	
	// initialise variables
	float h = 1.0f / N;
	int i, j, size = (N + 2)*(N + 2);
	float dv_dx, du_dy, dvort_dx, dvort_dy, Nx, Ny;

	// allocate memory for vorticity field
	float * vorticity;
	vorticity = new float[size];

	// TODO: MISSING exception handler?

	// compute vorticity = nabla x (u,v,0)
	FOR_EACH_CELL
		dv_dx = (v[IX(i + 1, j)] - v[IX(i - 1, j)]) / (2*h); // derivative of v in x-direction
		du_dy = (u[IX(i, j + 1)] - u[IX(i, j - 1)]) / (2*h); // derivative of u in y-direction
		vorticity[IX(i, j)] = dv_dx - du_dy;
	END_FOR
	// set boundaries to be equal to neighbouring cells
	set_bnd(N, 0, vorticity, solid);

	// compute and add vorticity confinement velocity per cell, via force
	float epsilontest = 0.00001f;
	FOR_EACH_CELL
		dvort_dx = (std::fabs(vorticity[IX(i - 1, j)]) - std::fabs(vorticity[IX(i + 1, j)])) / (2*h);
		dvort_dy = (std::fabs(vorticity[IX(i, j - 1)]) - std::fabs(vorticity[IX(i, j + 1)])) / (2*h);
		//Nx = (dvort_dx == 0 && dvort_dy == 0) ? 0 : dvort_dx / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy); // this is unstable
		//Ny = (dvort_dx == 0 && dvort_dy == 0) ? 0 : dvort_dy / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy); // this is unstable
		Nx = (dvort_dx*dvort_dx < epsilontest && dvort_dy*dvort_dy < epsilontest) ? 0 : dvort_dx / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy);
		Ny = (dvort_dx*dvort_dx < epsilontest && dvort_dy*dvort_dy < epsilontest) ? 0 : dvort_dy / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy);
		u[IX(i, j)] += eps*h*Ny*vorticity[IX(i, j)]*dt; // v_conf = f_conf * dt
		v[IX(i, j)] -= eps*h*Nx*vorticity[IX(i, j)]*dt; // f_conf = eps*h*(N x vorticity)
	END_FOR
}


/* public functions: */
void Solver::rigidbodySolve()
{
	// loop through rbodies and compute forces
	for (Force *f : m_forces) {
		f->calculateForce();
	}

	// loop through rbodies and user integrator
	for (RigidBody *rb : m_rbodies) {
		// save previous(current) state
		rb->m_PreviousState = rb->getState();

		// do next step
		m_Integrator->integrate(rb, dtrb);
		/* printf("m_Velocity: (%f, %f)\n", rb->m_Velocity[0], rb->m_Velocity[1]); */
		/* printf("m_Position: (%f, %f)\n", rb->m_Position[0], rb->m_Position[1]); */
		/* printf("m_Force: (%f, %f)\n", rb->m_Force[0], rb->m_Force[1]); */
	}

	// check collision test
	if (colsolver.detectCollision(m_rbodies)) {
		getPointOfCollision(dtrb);
	}
}

void Solver::getPointOfCollision(double timeStep)
{
	if (colsolver.detectCollision(m_rbodies)) {
		// 1 set state back to previous state
		// 2 do half a time step
		// 3 check if within tolerance of floor ---> done
		// 4 check for :
		// 	collision again
		// 	no collision

		/* //BACKUP
		double tc = timeStep/2; // start in the middle (half a time step)
		for (int i = 2; i <= 4; i++) {
			// 1 set state back to previous state
			for (RigidBody *rb : m_rbodies) {
				rb->setState(rb->m_PreviousState);
				// 2 do half a time step
				m_Integrator->integrate(rb, tc);
			}
			int divisor = std::pow(2, i);
			if (colsolver.detectCollision(m_rbodies)) {
				tc -= timeStep/divisor; 
			} else {
				tc += timeStep/divisor;
			}
		}
		*/

		// 1 determine middle step
		// 2 
		int i = 2;
		double tc = timeStep/2;
		do {
			int divisor = std::pow(2, i++);
			if (colsolver.detectCollision(m_rbodies)) {
				tc -= timeStep/divisor; 
			} else {
				tc += timeStep/divisor;
			}
			for (RigidBody *rb : m_rbodies) {
				rb->setState(rb->m_PreviousState);
				// 2 do half a time step
				m_Integrator->integrate(rb, tc);
			}
		} while(!colsolver.checkWithinTolerance());
		for (auto  &intervals : colsolver.overlapping_rbs) {
			printf("collision at:\n");
			printf("(%f, %f)\n", intervals.second[0].si, intervals.second[2].si);
		}
	}
}

void Solver::drawRigidBodies()
{
	for (RigidBody *rb : m_rbodies)
		rb->draw();
}

void Solver::addRigidBody(RigidBody *rb)
{
	m_rbodies.push_back(rb);
}

void Solver::addForce(Force *f)
{
	m_forces.push_back(f);
}

void Solver::setIntegrator(Integrator *i)
{
	if (!i)
		return;

	delete m_Integrator;
	m_Integrator = i;
	std::cout << "Integrator switched to: " << i->getString() << std::endl;
}

void Solver::dens_step ( int N, float * x, float * x0, float * u, float * v, int * solid )
{
	add_source ( N, x, x0);
	SWAP ( x0, x ); diffuse ( N, 0, x, x0, solid);
	SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, solid);
}

void Solver::vel_step ( int N, float * u, float * v, float * u0, float * v0, int * solid)
{
	add_source ( N, u, u0); add_source ( N, v, v0);
	confine_vorticity(N, u, v, solid);
	SWAP ( u0, u ); diffuse ( N, 1, u, u0, solid);
	SWAP ( v0, v ); diffuse ( N, 2, v, v0, solid);
	project ( N, u, v, u0, v0, solid );
	SWAP ( u0, u ); SWAP ( v0, v );
	advect ( N, 1, u, u0, u0, v0, solid ); advect ( N, 2, v, v0, u0, v0, solid );
	project ( N, u, v, u0, v0, solid );
	
}
