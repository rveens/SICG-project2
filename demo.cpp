/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
   Author : Jos Stam (jstam@aw.sgi.com)
   Creation Date : Jan 9 2003

   Description:

	This code is a simple prototype that demonstrates how to use the
	code provided in my GDC2003 paper entitles "Real-Time Fluid Dynamics
	for Games". This code uses OpenGL and GLUT for graphics and interface

  =======================================================================
*/

#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>

#include "Solver.h"
#include "RigidBodySquare.h"
#include "GravityForce.h"
#include "EulerStep.h"
#include "MidpointStep.h"
#include "RungeKuttaStep.h"
#include "SpringForce.h"

#include "Eigen/Dense"
#include "AntTweakBar.h"

/* macros */

#define IX(i,j) ((i)+(N+2)*(j))

/* global variables */
static Solver *solver;

static int N;
static float force, source;
static int dvel;
static bool mouse_drag = false; // whether mouse was dragged previous timestep

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static int * solid; // whether cell contains air/smoke/fluid or is solid

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;


/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/


static void free_data ( void )
{
	if ( u ) free ( u );
	if ( v ) free ( v );
	if ( u_prev ) free ( u_prev );
	if ( v_prev ) free ( v_prev );
	if ( dens ) free ( dens );
	if ( dens_prev ) free ( dens_prev );
	if ( solid ) free ( solid );
}

static void clear_data ( void )
{
	int i, size=(N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
	}
}

static void clear_solid_data(void) // needs to be cleared separately, for press C event
{
	int i, size = (N + 2)*(N + 2);

	for (i = 0; i<size; i++) {
		solid[i] = 0;
	}
}

static int allocate_data ( void )
{
	int size = (N+2)*(N+2);

	u			= (float *) malloc ( size*sizeof(float) );
	v			= (float *) malloc ( size*sizeof(float) );
	u_prev		= (float *) malloc ( size*sizeof(float) );
	v_prev		= (float *) malloc ( size*sizeof(float) );
	dens		= (float *) malloc ( size*sizeof(float) );	
	dens_prev	= (float *) malloc ( size*sizeof(float) );
	solid	    = (int *) malloc ( size*sizeof(int) );

	if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev || !solid ) {
		fprintf ( stderr, "cannot allocate data\n" );
		return ( 0 );
	}

	return ( 1 );
}

/*
----------------------------------------------------------------------
set the locations of solid cells with borders

+++++++++++++++++++++       ++    |    ++          +++++++
+++++++++++++++++++++       ++ 4  |  6 ++          +++++++    <---- border
++     |     |     ++     ++++    |    ++++
++  1  |  2  |  3  ++     ++++----+----++++
++     |     |     ++        |    |    |            0  = normal cell, "air", where smoke/fluid can reach
++-----+-----+-----++      2 | 10 | 11 | 2
++     |     |     ++        |    |    |
++  4  |  5  |  6  ++     ---+----+----+---
++     |     |     ++        |    |    |
++-----+-----+-----++      8 | 12 | 13 | 8
++     |     |     ++        |    |    |
++  7  |  8  |  9  ++     ++++----+----++++
++     |     |     ++     ++++    |    ++++
+++++++++++++++++++++       ++ 4  |  6 ++
+++++++++++++++++++++       ++    |    ++

----------------------------------------------------------------------
*/

/* set outside boundary of solids, of thickness t+1 */
static void set_solid_boundary(int t)
{
	int i, j;
	
	// set left/right/top/bottom boundary
	for ( i=t+1 ; i<=N-t ; i++ ) {
		solid[IX(t,i)] = 6; // left boundary: border to the right
		solid[IX(N+1-t,i)] = 4; // right boundary: border to the left
		solid[IX(i,t)] = 2; // bottom boundary: border above
		solid[IX(i,N+1-t)] = 8; // top boundary: border below
	}
	// corners: solid - no border
	solid[IX(t, t)] = 11; // left bottom boundary corner
	solid[IX(t, N + 1 - t)] = 13; // left top boundary corner
	solid[IX(N + 1 - t, t)] = 10; // right bottom boundary corner
	solid[IX(N + 1 - t, N + 1 - t)] = 12; // right top boundary corner

	// set solids without border around boundary (only if t>0)
	for ( i=0; i<t; i++ ){
		for ( j=i ; j<=N+1-i ; j++ ){
			solid[IX(i, j)] = 5;
			solid[IX(N+1-i, j)] = 5;
			solid[IX(j, i)] = 5;
			solid[IX(j, N+1-i)] = 5;
		}
	}
}

/* set centered inside square boundary of solids, distance t+1 away from outer boundary */
// WARNING creates unexpected velocity vortex, do not use
static void set_solid_square_center(int t)
{
	if (t > N/2 - 1) return; // distance from boundary must be less than half the total boundary length
	
	int i, j;

	// set left/right/top/bottom boundary
	for (i = t + 1; i <= N - t; i++) {
		solid[IX(t, i)] = 4; // left border
		solid[IX(N + 1 - t, i)] = 6; // right border
		solid[IX(i, t)] = 8; // bottom border
		solid[IX(i, N + 1 - t)] = 2; // top border
	}
	// corners: solid - no border
	solid[IX(t, t)] = 5;// 7; // left bottom
	solid[IX(t, N + 1 - t)] = 5;// 1; // left top
	solid[IX(N + 1 - t, t)] = 5;// 9; // right bottom
	solid[IX(N + 1 - t, N + 1 - t)] = 5;// 3; // right top

	// set solids without border inside square
	for (i = t; i<=N/2; i++){
		for (j = i; j <= N + 1 - i; j++){
			solid[IX(i, j)] = 5;
			solid[IX(N + 1 - i, j)] = 5;
			solid[IX(j, i)] = 5;
			solid[IX(j, N + 1 - i)] = 5;
		}
	}
}


/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	glutSwapBuffers ();
}

static void draw_velocity ( void )
{
	int i, j;
	float x, y, h;

	h = 1.0f/N;

	glColor3f ( 1.0f, 0.0f, 0.0f );
	glLineWidth ( 1.0f );

	glBegin ( GL_LINES );

		for ( i=1 ; i<=N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=1 ; j<=N ; j++ ) {
				y = (j-0.5f)*h;

				glVertex2f ( x, y );
				glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
			}
		}

	glEnd ();
}

static void draw_density ( void )
{
	int i, j;
	float x, y, h, d00, d01, d10, d11;

	h = 1.0f/N;

	glBegin ( GL_QUADS );

		for ( i=0 ; i<=N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=0 ; j<=N ; j++ ) {
				y = (j-0.5f)*h;

				d00 = dens[IX(i,j)];
				d01 = dens[IX(i,j+1)];
				d10 = dens[IX(i+1,j)];
				d11 = dens[IX(i+1,j+1)];

				glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
				glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
				glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
				glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
			}
		}

	glEnd ();
}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI ( float * d, float * u, float * v, int * solid )
{
	int i, j, size = (N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = d[i] = 0.0f;
	}

	if (!mouse_down[0] && !mouse_down[2]) {
		mouse_drag = false;
		return;
	}

	i = (int)((       mx /(float)win_x)*N+1);
	j = (int)(((win_y-my)/(float)win_y)*N+1);

	if ( solid[IX(i,j)] != 0 ) return;

	if ( mouse_down[0] && mouse_drag ) {
		u[IX(i, j)] = force * (mx - omx);
		v[IX(i, j)] = force * (omy - my);
	}

	mouse_drag = true;

	if ( mouse_down[2] ) {
		d[IX(i,j)] = source;
	}

	omx = mx;
	omy = my;

	return;
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 'c':
		case 'C':
			clear_data ();
			break;

		case 'q':
		case 'Q':
			free_data ();
			exit ( 0 );
			break;

		case 'v':
		case 'V':
			dvel = !dvel;
			break;
		case '1':
			if (solver)
			solver->setIntegrator(new EulerStep());
			break;
		case '2':
			solver->setIntegrator(new MidpointStep());
			break;
		case '3':
			solver->setIntegrator(new RungeKuttaStep());
			break;
	}

	TwEventKeyboardGLUT(key, x, y);
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	mouse_down[button] = state == GLUT_DOWN;

	TwEventMouseButtonGLUT(button, state, x, y);
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;


	if (mouse_down[1]) {
		int i = (int)((mx / (float)win_x)*N + 1);
		int j = (int)(((win_y - my) / (float)win_y)*N + 1);
		// 0) get mouse postion in world coordinates
		double x = (double)i / (double)N;
		double y = (double)j / (double)N;

		// 1) search rigid body on mouse position
		RigidBody *rb = solver->getRigidBodyOnMousePosition(x, y);
		if (rb != nullptr) {
			// 2) set the new position of the rigid body
			rb->m_Position = Vector2d(x, y);
		}
	}

	TwEventMouseMotionGLUT(x, y);
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;

	TwWindowSize(width, height);
}

static void idle_func ( void )
{
	get_from_UI ( dens_prev, u_prev, v_prev, solid );
	solver->vel_step ( N, u, v, u_prev, v_prev, solid );
	solver->dens_step ( N, dens, dens_prev, u, v, solid );
	solver->rigidbodySolve(N, u, v, solid);

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	if (dvel) {
		draw_density();
		draw_velocity();
	}
	else {
		draw_density();
	}
	solver->drawObjects(N);

	TwDraw();

	post_display ();
}


/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Alias | wavefront" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


void setupAntTweakBar()
{
	TwInit(TW_OPENGL, NULL);

	// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	// - Directly redirect GLUT special key events to AntTweakBar
	glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);
	// - Send 'glutGetModifers' function pointer to AntTweakBar;
	//   required because the GLUT key event functions do not report key modifiers states.
	TwGLUTModifiersFunc(glutGetModifiers);

	// Create a tweak bar
	TwBar *bar = TwNewBar("TweakBar");
	TwDefine(" GLOBAL help='This example shows how to integrate AntTweakBar with GLUT and OpenGL.' "); // Message added to the help bar.
	TwDefine(" TweakBar size='200 200' color='96 216 224' "); // change default tweak bar size and color

	TwAddVarRW(bar, "Boundingbox", TW_TYPE_BOOLCPP, &solver->m_Drawbb, " group='Draw'");
	TwAddVarRW(bar, "Boundingbox cell-aligned", TW_TYPE_BOOLCPP, &solver->m_DrawbbCells, " group='Draw'");
	TwAddVarRW(bar, "Voxelize", TW_TYPE_BOOLCPP, &solver->m_DrawbbCellsOccupied, " group='Draw'");
	TwAddVarRW(bar, "Pushed cells", TW_TYPE_BOOLCPP, &solver->m_DrawPushFluidCells, " group='Draw'");

	/*
	// rb one
	Matrix2d rot = Matrix2d::Identity();
	rot(0, 0) = 0.7071;
	rot(0, 1) = -0.7071;
	rot(1, 0) = 0.7071;
	rot(1, 1) = 0.7071;
	Vector2d init_position(0.6, 0.6);
	Vector2d rb_size(0.2, 0.2);
	RigidBody *rb = new RigidBodySquare(init_position, rb_size, 1, rot);
	solver->addRigidBody(rb);
	solver->addForce(new GravityForce(rb));

	// rb two
	Matrix2d rot2 = Matrix2d::Identity();
	Vector2d init_position2(0.799, 0.799);
	Vector2d rb_size2(0.2, 0.2);
	RigidBody *rb2 = new RigidBodySquare(init_position2, rb_size2, 1, rot2);
	solver->addRigidBody(rb2);
	solver->addForce(new GravityForce(rb2));

	// particle 1
	Particle *p = new Particle(Vector2d(0.1, 0.8), 1);
	solver->addParticle(p);
	solver->addForce(new GravityForce(p));
	*/
}


/*
----------------------------------------------------------------------
particles
----------------------------------------------------------------------
*/

static void create_rectangular_cloth(int w, int h, double gridSize, double startx, double starty, double pMass)
{
	if (w < 2 || h < 2) {
		printf("Only Rectangular cloth of 2x2 or higher supported.");
		exit(0);
	}

	Particle *** p_storage;

	// Allocate memory
	p_storage = new Particle**[h+1];
	for (int i = 0; i < h+1; ++i)
		p_storage[i] = new Particle*[w+1];
	
	//Particle *p_storage[h + 1][w + 1];	// 2x2 'grid' results in a 3x3 particle system

										// create particles
	for (int j = 0; j <= h; j++) {
		for (int i = 0; i <= w; i++) {
			p_storage[j][i] = new Particle(Vector2d(startx + i*gridSize, starty - j*gridSize), pMass);
			solver->addParticle(p_storage[j][i]);
		}
	}

	double ks_struct = 50.0, kd_struct = 5.0;
	// create structural springs
	for (int j = 0; j <= h; j++) {
		for (int i = 0; i <= w; i++) {
			if (i < w)
				solver->addForce(new SpringForce(p_storage[j][i], p_storage[j][i + 1], gridSize, ks_struct, kd_struct));
			if (j < h)
				solver->addForce(new SpringForce(p_storage[j][i], p_storage[j + 1][i], gridSize, ks_struct, kd_struct));
		}
	}

	double ks_shear = 50.0, kd_shear = 5.0;
	// create shear springs
	for (int j = 0; j < h; j++) {
		for (int i = 0; i < w; i++) {
			solver->addForce(new SpringForce(p_storage[j][i], p_storage[j + 1][i + 1], gridSize*sqrt(2), ks_shear, kd_shear));
			solver->addForce(new SpringForce(p_storage[j][i + 1], p_storage[j + 1][i], gridSize*sqrt(2), ks_shear, kd_shear));
		}
	}

	double ks_flexion = 50.0, kd_flexion = 5.0;
	// create flexion springs
	for (int j = 0; j <= h; j++) {
		for (int i = 0; i <= w; i++) {
			if (i < w - 1)
				solver->addForce(new SpringForce(p_storage[j][i], p_storage[j][i + 2], gridSize * 2, ks_flexion, kd_flexion));
			if (j < h - 1)
				solver->addForce(new SpringForce(p_storage[j][i], p_storage[j + 2][i], gridSize * 2, ks_flexion, kd_flexion));
		}
	}


	// add gravity
	for (int j = 0; j <= h; j++) {
		for (int i = 0; i <= w; i++) {
			solver->addForce(new GravityForce(p_storage[j][i]));
		}
	}

	// top row fixed-point constraint
	for (int i = 0; i <= w; i++) {
		const double dist = 0.1;
		Vector2d center = p_storage[0][i]->m_Position;
		Vector2d offset(dist, p_storage[0][i]->m_Position[1]);

		/* solver->addConstraint(new CircularWireConstraint(p_storage[0][i], Vec2(center[0], center[1] + dist), dist)); */
		p_storage[0][i]->m_Static = true;
	}

	/*
	// add wind 
	for (int j = 0; j <= h; j++) {
		for (int i = 0; i <= w; i++) {
			solver->addForce(new WindForce(p_storage[j][i], Vector2d(2.0, 0)));
		}
	}
	*/
}

/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	float dt;
	float diff;
	float visc;
	float vort;

	if ( argc != 1 && argc != 7 ) {
		fprintf ( stderr, "usage : %s N dt diff visc force source vort\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
		fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
		fprintf ( stderr, "\t vort   : strength of vorticity confinement\n" );
		exit ( 1 );
	}

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		diff = 0.00001f; // was 0
		visc = 0.001f; // was 0
		force = 1.0f; // was 5
		source = 100.0f;
		vort = 5.0f; // influence of vorticity confinement
		fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g vort=%g\n",
			N, dt, diff, visc, force, source, vort );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		diff = atof(argv[3]);
		visc = atof(argv[4]);
		force = atof(argv[5]);
		source = atof(argv[6]);
		vort = atof(argv[7]);
	}

	/* init stuff */
	solver = new Solver(dt, 0.001, diff, visc, vort);
	setupAntTweakBar();

	// cloth 1
	create_rectangular_cloth(10, 10, 0.05, 0.1, 0.9, 0.1);

	/* end init stuff */
	

	printf ( "\n\nHow to use this demo:\n\n" );
	printf ( "\t Add densities with the right mouse button\n" );
	printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
	printf ( "\t Toggle density/velocity display with the 'v' key\n" );
	printf ( "\t Clear the simulation by pressing the 'c' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dvel = 0;

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();
	clear_solid_data();
	set_solid_boundary(5);
	set_solid_square_center(25);

	win_x = 720;
	win_y = 720;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}
