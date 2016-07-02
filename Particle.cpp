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

void Particle::setState(const VectorXd &state)
{
	this->m_Position[0] = state[0];
	this->m_Position[1] = state[1];

	this->m_Velocity[0] = state[2];
	this->m_Velocity[1] = state[3];
}

VectorXd Particle::getState()
{
	VectorXd vector;
	vector[0] = m_Position[0];
	vector[1] = m_Position[1];
	vector[2] = m_Velocity[0];
	vector[3] = m_Velocity[1];

	return vector;
}

VectorXd Particle::derivEval()
{
	Vector2d f_by_m = m_Force/(double)m_Mass;

	VectorXd der = VectorXd::Zero(4);
	int i = 0;

	der(i++) = m_Velocity[0];
	der(i++) = m_Velocity[1];
	der(i++) = f_by_m[0];
	der(i++) = f_by_m[1];

    return der;
}
	
VectorXd Particle::derivEval(const VectorXd &input)
{
	VectorXd der = VectorXd::Zero(4);
	int i = 0;

	der(i++) = input[2];
	der(i++) = input[3];
	der(i++) = m_Force[0] / m_Mass;
	der(i++) = m_Force[1] / m_Mass;

	return der;
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