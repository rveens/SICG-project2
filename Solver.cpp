#include "Solver.h"
#include "RungeKuttaStep.h"
#include <GL/glut.h>

#include <cmath>
#include <iostream>

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) { if (solid[((i)+(N+2)*(j))]==0) {
#define END_FOR }}}


Solver::Solver(float _dtfluid, float _dtrb, float _diff, float _visc, float _vort) :
	m_Integrator(new RungeKuttaStep()), dt(_dtfluid), dtrb(_dtrb),
	diff(_diff), visc(_visc), vort(_vort)
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


void Solver::lin_solve ( int N, int b, float * x, float * x0, float a, float c, int * solid )
{
	int i, j, k;
	float x_left, x_right, x_up, x_down;

	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x_left  = (solid[IX(i-1,j)]!=0) ? ((b==1) ? -x[IX(i,j)] : x[IX(i,j)]) : x[IX(i-1,j)];
			x_right = (solid[IX(i+1,j)]!=0) ? ((b==1) ? -x[IX(i,j)] : x[IX(i,j)]) : x[IX(i+1,j)];
			x_down  = (solid[IX(i,j-1)]!=0) ? ((b==2) ? -x[IX(i,j)] : x[IX(i,j)]) : x[IX(i,j-1)];
			x_up    = (solid[IX(i,j+1)]!=0) ? ((b==2) ? -x[IX(i,j)] : x[IX(i,j)]) : x[IX(i,j+1)];
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x_left + x_right + x_down + x_up))/c;
		END_FOR
		//set_bnd ( N, b, x, solid );
	}
}

void Solver::diffuse ( int N, int b, float * x, float * x0, float diffvisc, int * solid)
{
	float a=dt*diffvisc*N*N;
	lin_solve ( N, b, x, x0, a, 1+4*a, solid );
}

/*
u,v: velocity field (hor/vert) that causes advection
d0: field to be advected | d: newly advected field
*/
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
	//set_bnd ( N, b, d, solid );
}

void Solver::project ( int N, float * u, float * v, float * p, float * div, int * solid )
{
	int i, j;
	float u_left, u_right, v_up, v_down, p_left, p_right, p_up, p_down;

	FOR_EACH_CELL
		u_right = (solid[IX(i+1,j)]!=0) ? -u[IX(i,j)] : u[IX(i+1,j)];
		u_left  = (solid[IX(i-1,j)]!=0) ? -u[IX(i,j)] : u[IX(i-1,j)];
		v_up    = (solid[IX(i,j+1)]!=0) ? -v[IX(i,j)] : v[IX(i,j+1)];
		v_down  = (solid[IX(i,j-1)]!=0) ? -v[IX(i,j)] : v[IX(i,j-1)];
		div[IX(i,j)] = -0.5f*(u_right - u_left + v_up - v_down)/N;
		p[IX(i,j)] = 0;
	END_FOR	
	//set_bnd ( N, 0, div, solid ); set_bnd ( N, 0, p, solid );

	lin_solve ( N, 0, p, div, 1, 4, solid );

	FOR_EACH_CELL
		p_right = (solid[IX(i+1,j)]!=0) ? 0 : p[IX(i+1,j)];
		p_left  = (solid[IX(i-1,j)]!=0) ? 0 : p[IX(i-1,j)];
		p_up    = (solid[IX(i,j+1)]!=0) ? 0 : p[IX(i,j+1)];
		p_down  = (solid[IX(i,j-1)]!=0) ? 0 : p[IX(i,j-1)];
		u[IX(i,j)] -= 0.5f*N*(p_right-p_left);
		v[IX(i,j)] -= 0.5f*N*(p_up-p_down);
	END_FOR
	//set_bnd ( N, 1, u, solid ); set_bnd ( N, 2, v, solid );
}

void Solver::confine_vorticity(int N, float * u, float * v, int * solid)
{
	// initialise variables
	float h = 1.0f / N;
	int i, j, size = (N + 2)*(N + 2);
	float dv_dx, du_dy, dvort_dx, dvort_dy, Nx, Ny, vort_left, vort_right, vort_up, vort_down;

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
	//set_bnd(N, 0, vorticity, solid);

	// compute and add vorticity confinement velocity per cell, via force
	float epsilontest = 0.00001f;
	FOR_EACH_CELL
		vort_left  = (solid[IX(i-1,j)]!=0) ? vorticity[IX(i,j)] : vorticity[IX(i-1,j)];
		vort_right = (solid[IX(i+1,j)]!=0) ? vorticity[IX(i,j)] : vorticity[IX(i+1,j)];
		vort_down  = (solid[IX(i,j-1)]!=0) ? vorticity[IX(i,j)] : vorticity[IX(i,j-1)];
		vort_up    = (solid[IX(i,j+1)]!=0) ? vorticity[IX(i,j)] : vorticity[IX(i,j+1)];
		dvort_dx = (std::fabs(vort_left) - std::fabs(vort_right)) / (2*h);
		dvort_dy = (std::fabs(vort_down) - std::fabs(vort_up)) / (2*h);
		//Nx = (dvort_dx == 0 && dvort_dy == 0) ? 0 : dvort_dx / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy); // this is unstable
		//Ny = (dvort_dx == 0 && dvort_dy == 0) ? 0 : dvort_dy / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy); // this is unstable
		Nx = (dvort_dx*dvort_dx < epsilontest && dvort_dy*dvort_dy < epsilontest) ? 0 : dvort_dx / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy);
		Ny = (dvort_dx*dvort_dx < epsilontest && dvort_dy*dvort_dy < epsilontest) ? 0 : dvort_dy / std::sqrt(dvort_dx*dvort_dx + dvort_dy*dvort_dy);
		u[IX(i, j)] += vort*h*Ny*vorticity[IX(i, j)]*dt; // v_conf = f_conf * dt
		v[IX(i, j)] -= vort*h*Nx*vorticity[IX(i, j)]*dt; // f_conf = eps*h*(N x vorticity)
	END_FOR
}


double Solver::count_density(int N, float * x, int * solid) {
	double total_dens = 0;
	int i, j;
	FOR_EACH_CELL
		total_dens += x[IX(i, j)];
	END_FOR
	return total_dens;
}

void Solver::preserve_density(int N, float * x, int * solid, double old_density, double new_density) {
	if (new_density == 0) return;
	double difference = old_density - new_density;
	int i, j;
	// spread out density difference proportionally over density field
	FOR_EACH_CELL
		x[IX(i, j)] += difference * x[IX(i, j)] / new_density;
	END_FOR
}


/* public functions: */
void Solver::rigidbodySolve(int N, float * u, float * v, int *solid, float *dens, float * p, float * div)
{
	double vel_friction = 0.95; // must be <= 1
	double ang_friction = 0.9;
	// 0. apply friction to velocities and momentums
	for (RigidBody *rb : m_rbodies) {
		rb->m_LinearMomentum *= vel_friction;
		rb->m_AngularMomentum *= ang_friction;
	}
	for (Particle *p : m_particles) {
		p->m_Velocity *= vel_friction;
	}

	// 1. set forces to zero
	for (RigidBody *rb : m_rbodies) {
		rb->m_Force = Vector2d(0.0, 0.0);
		rb->m_Torque = 0;
	}
	for (Particle *p : m_particles) {
		p->m_Force = Vector2d(0.0, 0.0); // dit hoeft in principe niet meer, na fluid forces
	}

	// 1.1 apply fluid forces to particles from velocity field
	for (Particle *p : m_particles) {
		double x, y, s0, s1, t0, t1;
		int i0, i1, j0, j1;
		x = (p->m_Position[0])*(N+2);
		y = (p->m_Position[1])*(N+2);
		i0 = (int) x;
		j0 = (int) y;
		if (i0<0) i0 = 0; if (i0>N) i0 = N;
		if (j0<0) j0 = 0; if (j0>N) j0 = N;
		i1 = i0 + 1;
		j1 = j0 + 1;
		s1 = x - i0; s0 = 1 - s1;
		t1 = y - j0; t0 = 1 - t1;
		p->m_Force[0] = (s0*(t0*u[IX(i0, j0)] + t1*u[IX(i0, j1)]) +
			s1*(t0*u[IX(i1, j0)] + t1*u[IX(i1, j1)])) / (dt * p->m_Mass);
		p->m_Force[1] = (s0*(t0*v[IX(i0, j0)] + t1*v[IX(i0, j1)]) +
			s1*(t0*v[IX(i1, j0)] + t1*v[IX(i1, j1)])) / (dt * p->m_Mass);
	}

	// 1.2 apply fluid forces and torques to rigid bodies from velocity field
	double fluidforce = 25;
	double fluidtorque = 0.05;
	for (RigidBody *rb : m_rbodies) {
		double force_x, force_y;
		rb->getBoundaryCells(N, solid);
		for (std::array<int, 2> cell : rb->gridIndicesCloseToBoundary) {
			force_x = u[IX(cell[0], cell[1])] / dt;
			force_y = v[IX(cell[0], cell[1])] / dt;
			rb->m_Force[0] += fluidforce * force_x;
			rb->m_Force[1] += fluidforce * force_y;
			// torque = (r - center) x Force
			rb->m_Torque += fluidtorque * ( (cell[0] - rb->m_Position[0])*force_y - (cell[1] - rb->m_Position[1])*force_x );
		}
	}

	// 2. loop through objects and compute forces
	for (Force *f : m_forces) {
		f->calculateForce();
	}

	// 2.1 turn off rigid body solids
	for (RigidBody *rb : m_rbodies) {
		for (Vector2i &index : rb->gridIndicesOccupied) {
			if (solid[IX(index[0], index[1])] != 2) {
				solid[IX(index[0], index[1])] = 0;
			}
		}
	}

	// 3. loop through rbodies and user integrator
	for (RigidBody *rb : m_rbodies) {
		// save previous(current) state
		rb->m_PreviousState = rb->getState();
		// do next step
		m_Integrator->integrate(rb, dtrb);
	}
	for (Particle *p : m_particles) {
		m_Integrator->integrate(p, dtrb);
	}

	// 3.1 set static particles
	for (Particle *p : m_particles) {
		if (p->m_Static)
			p->reset();
	}
	
	// 4. voxelize rbodies
	for (RigidBody *rb : m_rbodies) {
		rb->voxelize(N);
		// 4.1 turn on rigid body solids
		for (Vector2i &index : rb->gridIndicesOccupied) {
			if (solid[IX(index[0], index[1])] != 2) {
				solid[IX(index[0], index[1])] = 1;
				// set velocity to 0 under rigid body
				u[IX(index[0], index[1])] = 0;
				v[IX(index[0], index[1])] = 0;
			}
		}
	}

	// 5. push density OLD
	/*
	for (RigidBody *rb : m_rbodies) {
		// for each boundary cell
		for (Vector2i &cell : rb->getBoundaryCells(N, solid)) {
			// if there is density
			if (dens[IX(cell[0], cell[1])] > 0) {
				// find the neighbouring non-solid cells
				std::vector<Vector2i> neighbours;
				if (solid[IX(cell[0] - 1, cell[1])] == 0)
					neighbours.push_back(Vector2i(cell[0] - 1, cell[1]));
				if (solid[IX(cell[0], cell[1] - 1)] == 0)
					neighbours.push_back(Vector2i(cell[0], cell[1] - 1));
				if (solid[IX(cell[0] + 1, cell[1])] == 0)
					neighbours.push_back(Vector2i(cell[0] + 1, cell[1]));
				if (solid[IX(cell[0], cell[1] + 1)] == 0)
					neighbours.push_back(Vector2i(cell[0], cell[1] + 1));

				// if there are no non-solid neighbours, we cannot push density!
				if (neighbours.empty()) {
					printf("Density cannot escape!\n");
				}

				// distribute the density to neighbours, evenly
				double density = dens[IX(cell[0], cell[1])];
				density /= neighbours.size();
				for (Vector2i &neighbour : neighbours) {
					dens[IX(neighbour[0], neighbour[1])] += density;
				}
				dens[IX(cell[0], cell[1])] = 0;
			}
		}
	}
	*/

	
	
	// 5. push density (using previous state of rigid body)
	for (RigidBody *rb : m_rbodies) {
		// compute transformation matrix: T(center_new)*R(rotation)*T(-center_old)
		// translation matrix from origin to new center
		Matrix3d T_new;
		T_new << 1 , 0 , rb->m_Position[0],
			     0 , 1 , rb->m_Position[1],
			     0 , 0 , 1;
		// rotation matrix from old to new angle
		Matrix3d R_total, R_new, R_old;
		R_new << rb->m_Rotation(0,0), rb->m_Rotation(0,1), 0,
			     rb->m_Rotation(1,0), rb->m_Rotation(1,1), 0,
			     0                  , 0                  , 1;
		R_old << rb->m_PreviousState[2], rb->m_PreviousState[3], 0,
			     rb->m_PreviousState[4], rb->m_PreviousState[5], 0,
			     0                     , 0                     , 1;
		R_total = R_new * R_old.transpose();
		// translation matrix from old center to origin
		Matrix3d T_old;
		T_old << 1 , 0 , -rb->m_PreviousState[0],
			     0 , 1 , -rb->m_PreviousState[1],
			     0 , 0 , 1;
		// total transformation matrix
		Matrix3d transformation = T_new*R_total*T_old;
		// for each cell
		double leftover_density = 0;
		for (Vector2i &cell : rb->gridIndicesOccupied) {
			// if there is density
			if (dens[IX(cell[0], cell[1])] > 0) {
				Vector3d cell_hom; // homogeneous coordinates
				cell_hom << cell[0], cell[1], 1;
				Vector3d cell_new_hom = transformation * cell_hom;
				// integer position of new cell (left bottom)
				Vector2i cell_new;
				cell_new << (int) cell_new_hom[0], (int) cell_new_hom[1];

				// TEST: draw line from old to new position
				//glPointSize(10);
				//glColor3f(0.f, 1.f, 1.f);
				//glBegin(GL_LINES);
				//glVertex2f(cell_hom[0] / (double) N, cell_hom[1] / (double)N);
				//glVertex2f(cell_new_hom[0] / (double)N, cell_new_hom[1] / (double)N);
				//glEnd();
				//std::cout << "Density moved from (" << std::to_string(cell[0]) << "," << std::to_string(cell[1])
				//	<< ") to (" + std::to_string(cell_new[0]) << "," << std::to_string(cell_new[1]) << ")\n";

				// find the neighbouring non-solid cells
				std::vector<Vector2i> neighbours;
				if (solid[IX(cell_new[0]  , cell_new[1]  )] == 0)
					neighbours.push_back(Vector2i(cell_new[0]  , cell_new[1]  ));
				if (solid[IX(cell_new[0]+1, cell_new[1]  )] == 0)
					neighbours.push_back(Vector2i(cell_new[0]+1, cell_new[1]  ));
				if (solid[IX(cell_new[0]  , cell_new[1]+1)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]  , cell_new[1]+1));
				if (solid[IX(cell_new[0]+1, cell_new[1]+1)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]+1, cell_new[1]+1));
				
				if (solid[IX(cell_new[0]-1, cell_new[1]  )] == 0)
					neighbours.push_back(Vector2i(cell_new[0]-1, cell_new[1]  ));
				if (solid[IX(cell_new[0]+2, cell_new[1]  )] == 0)
					neighbours.push_back(Vector2i(cell_new[0]+2, cell_new[1]  ));
				if (solid[IX(cell_new[0]-1, cell_new[1]+1)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]-1, cell_new[1]+1));
				if (solid[IX(cell_new[0]+2, cell_new[1]+1)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]+2, cell_new[1]+1));

				if (solid[IX(cell_new[0]  , cell_new[1]-1)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]  , cell_new[1]-1));
				if (solid[IX(cell_new[0]+1, cell_new[1]-1)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]+1, cell_new[1]-1));
				if (solid[IX(cell_new[0]  , cell_new[1]+2)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]  , cell_new[1]+2));
				if (solid[IX(cell_new[0]+1, cell_new[1]+2)] == 0)
					neighbours.push_back(Vector2i(cell_new[0]+1, cell_new[1]+2));
				
				// density to be pushed
				double density = dens[IX(cell[0], cell[1])] + leftover_density;
				leftover_density = 0;
				// if there are no non-solid neighbours, we cannot push density!
				if (neighbours.empty()) {
					//printf("Density cannot escape!\n");
					leftover_density = density;
				} else {
					// distribute the density to neighbours, evenly
					density /= neighbours.size();
					for (Vector2i &neighbour : neighbours) {
						dens[IX(neighbour[0], neighbour[1])] += density;
					}
				}
				// remove density
				dens[IX(cell[0], cell[1])] = 0;
			}
		}
		if (leftover_density != 0) {
			std::cout << "There is leftover density: " << std::to_string(leftover_density) << "\n";
			// spread out leftover density all around RB
			rb->getBoundaryCells(N, solid);
			if (rb->gridIndicesCloseToBoundary.size() == 0) {
				printf("Loss of density!\n");
			} else {
				leftover_density /= rb->gridIndicesCloseToBoundary.size();
				for (auto neighbour : rb->gridIndicesCloseToBoundary) {
					dens[IX(neighbour[0], neighbour[1])] += leftover_density;
				}
			}
		}
	}
	

	// 6. RB applies velocity to fluid
	double RBtofluid = 0.005;
	for (RigidBody *rb : m_rbodies) {
		rb->getBoundaryCells(N, solid);
		for (auto &cell : rb->gridIndicesCloseToBoundary) {
			Vector2d velocity = rb->getVelocity();
			u[IX(cell[0], cell[1])] += RBtofluid * rb->m_Mass * velocity[0] / dt;
			v[IX(cell[0], cell[1])] += RBtofluid * rb->m_Mass * velocity[1] / dt;
		}
	}

	// project velocities again to remain mass preserving
	project(N, u, v, p, div, solid);
	

	// check collision test
	/*if (colsolver.detectCollisionBroad(m_rbodies)) {
		for (auto pair : colsolver.overlapping_rbs) {
			RigidBody *rb1 = std::get<0>(pair.first);
			RigidBody *rb2 = std::get<1>(pair.first);
			bool narrowCol = colsolver.detectCollisionNarrow(rb1, rb2);
			//printf("narrowCheck: %d\n", narrowCol);
			if (narrowCol) {
				colsolver.findContactPoints(rb1, rb2);
			}
		}
	}*/
}

void Solver::drawObjects(int N, int *solid)
{
	for (RigidBody *rb : m_rbodies) {
		rb->draw(N);
		if (m_Drawbb)
			rb->drawbb();
		if (m_DrawbbCells)
			rb->drawbbCells(N);
		if (m_DrawbbCellsOccupied)
			rb->drawbbCellsOccupied(N);
		if (m_DrawBoundaries)
			rb->drawBoundaryCells(N, solid);
		if (m_DrawEdgeNormals)
			rb->drawEdgeNormals();
	}
		
	for (Particle *p : m_particles)
		p->draw();

	for (Force *f : m_forces)
		f->draw();
}

void Solver::addRigidBody(RigidBody *rb)
{
	m_rbodies.push_back(rb);
}

void Solver::addParticle(Particle *p)
{
	m_particles.push_back(p);
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

RigidBody *Solver::getRigidBodyOnMousePosition(double x, double y)
{
	// loop over the rigid bodies (m_rbodies) and 
	for (RigidBody *rb : m_rbodies) {
		// check if the given coordinates are in the bounding box of RigidBody 'rb' 
		std::vector<double> bb = rb->computeAABB();
		if (x > bb[0] && x < bb[2] &&
			y > bb[1] && y < bb[3]) {
			return rb;
		}
	}

	return nullptr;
}

void Solver::dens_step ( int N, float * x, float * x0, float * u, float * v, int * solid )
{
	add_source ( N, x, x0);
	double old_density = count_density(N, x, solid);
	std::cout << "Old density: " << std::to_string(old_density) << "\n";
	SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, solid);
	SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, solid);
	double new_density = count_density(N, x, solid);
	std::cout << "New density: " << std::to_string(new_density) << "\n";
	preserve_density(N, x, solid, old_density, new_density);
}

void Solver::vel_step ( int N, float * u, float * v, float * u0, float * v0, int * solid)
{
	add_source ( N, u, u0); add_source ( N, v, v0);
	SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, solid);
	SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, solid);
	project ( N, u, v, u0, v0, solid );
	SWAP ( u0, u ); SWAP ( v0, v );
	advect ( N, 1, u, u0, u0, v0, solid ); advect ( N, 2, v, v0, u0, v0, solid );
	project ( N, u, v, u0, v0, solid );
	confine_vorticity(N, u, v, solid);
	project ( N, u, v, u0, v0, solid );
}