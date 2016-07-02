#include "Particle.h"

#include <GL/glut.h>

Particle::Particle(const Vector2d & ConstructPos, int mass) :
	m_ConstructPos(ConstructPos), m_Position(ConstructPos), m_Velocity(Vector2d(0.0, 0.0)),
			m_Force(Vector2d(0.0, 0.0)), m_Mass(mass)
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vector2d(0.0, 0.0);
	m_Force = Vector2d(0.0, 0.0);
}
void Particle::draw()
{
	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}

unsigned int Particle::getDim()
{
	return 4;
}

void Particle::setState(const Vector4d &state)
{
	this->m_Position[0] = state[0];
	this->m_Position[1] = state[1];

	this->m_Velocity[0] = state[2];
	this->m_Velocity[1] = state[3];
}

Vector4d Particle::getState()
{
	Vector4d vector;
	vector[0] = m_Position[0];
	vector[1] = m_Position[1];
	vector[2] = m_Velocity[0];
	vector[3] = m_Velocity[1];

	return vector;
}

Vector4d Particle::derivEval(double timeStep)
{
	Vector2d f_by_m = m_Force/(double)m_Mass;
	Vector2d new_Velocity = m_Velocity + timeStep*f_by_m;

	/* return Vec4(new_Velocity[0], new_Velocity[1], f_by_m[0], f_by_m[1]); */
    return Vector4d(m_Velocity[0], m_Velocity[1], f_by_m[0], f_by_m[1]);
}
	
Vector4d Particle::derivEval(const Vector4d &input)
{
	return Vector4d(input[2], input[3], m_Force[0]/m_Mass, m_Force[1]/m_Mass);
}

Particle *Particle::checkSelected(double x_given, double y_given)
{
	// check if given position is within the square of 
	double x1 = m_Position[0]-h/2.0;
	double x2 = m_Position[0]+h/2.0;
	double y1 = m_Position[1]-h/2.0;
	double y2 = m_Position[1]+h/2.0;
	
	if (x_given >= x1 && x_given <= x2) {
		/* printf("x ok\n"); */
 		if (y_given >= y1 && y_given <= y2) {
			/* printf("y ok\n"); */
			/* printf("Particle geraakt\n"); */
			return this;
		}
	}
	return nullptr;
}
