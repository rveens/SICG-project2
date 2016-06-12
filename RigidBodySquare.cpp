#include "RigidBodySquare.h"
#include <GL/glut.h>

#include "Eigen/Dense"

RigidBodySquare::RigidBodySquare(const Vector2d & ConstructPos, Vector2d size, int mass,
		Matrix2d rotation) : RigidBody(ConstructPos, mass, Matrix2d::Zero(), Matrix2d::Zero(), rotation),
					m_Size(size)
{
	// calc Ibody
	m_Ibody(0, 0) = m_Size[0];
	m_Ibody(1, 1) = m_Size[1];
	m_Ibody *= 1.0/12.0;

	// calc IbodyInv
	m_IbodyInv = m_Ibody.inverse();
}

RigidBodySquare::~RigidBodySquare(void)
{

}

void RigidBodySquare::draw()
{
	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_QUADS);

	/* printf("1: (%f, %f)\n", m_Position[0], m_Position[1]); */
	/* printf("2: (%f, %f)\n", m_Position[0]+m_Size[0], m_Position[1]); */
	/* printf("3: (%f, %f)\n", m_Position[0]+m_Size[0], m_Position[1]+m_Size[1]); */
	/* printf("4: (%f, %f)\n", m_Position[0], m_Position[1]+m_Size[1]); */
	glVertex2f(m_Position[0], m_Position[1]);
	glVertex2f(m_Position[0]+m_Size[0], m_Position[1]);
	glVertex2f(m_Position[0]+m_Size[0], m_Position[1]+m_Size[1]);
	glVertex2f(m_Position[0], m_Position[1]+m_Size[1]);

	glEnd();
}
