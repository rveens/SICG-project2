#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(std::shared_ptr<Particle> p1, std::shared_ptr<Particle> p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

SpringForce::~SpringForce()
{

}

void SpringForce::draw()
{
  glLineWidth(3);
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

void SpringForce::calculateForce()
{
	Vector2d l 	= m_p1->m_Position - m_p2->m_Position;
	Vector2d l_dot 	= m_p1->m_Velocity - m_p2->m_Velocity;
	Vector2d l_normalized = l;
	l_normalized.normalize();

	Vector2d f_p1 = (m_ks*(l.norm() - m_dist) + m_kd*((l_dot.dot(l))/l.norm())) * (l / l.norm());
	Vector2d f_p2 = -f_p1;

	m_p1->m_Force += -f_p1;
	/* printf("force p1: %f, %f\n", m_p1->m_Force[0], m_p1->m_Force[1]); */
	m_p2->m_Force += -f_p2;
	/* printf("force p2: %f, %f\n", m_p2->m_Force[0], m_p2->m_Force[1]); */
}
