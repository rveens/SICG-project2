#include "RigidBodySquare.h"
#include <GL/glut.h>

#include "Eigen/Dense"

#include <iostream>

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

	std::cout << "ibody:" << std::endl;
	std::cout << m_Ibody << std::endl;

	std::cout << "ibodyInv:" << std::endl;
	std::cout << m_IbodyInv << std::endl;
}

RigidBodySquare::~RigidBodySquare(void)
{

}

void RigidBodySquare::draw()
{

	/* printf("1: (%f, %f)\n", m_Position[0], m_Position[1]); */
	/* printf("2: (%f, %f)\n", m_Position[0]+m_Size[0], m_Position[1]); */
	/* printf("3: (%f, %f)\n", m_Position[0]+m_Size[0], m_Position[1]+m_Size[1]); */
	/* printf("4: (%f, %f)\n", m_Position[0], m_Position[1]+m_Size[1]); */

	// m_Size[0] means x
	// m_Size[1] means y
	Vector2d bl = Vector2d(-m_Size[0]/2, -m_Size[1]/2);
	Vector2d br = Vector2d(+m_Size[0]/2, -m_Size[1]/2);
	Vector2d tr = Vector2d(+m_Size[0]/2, +m_Size[1]/2);
	Vector2d tl = Vector2d(-m_Size[0]/2, +m_Size[1]/2);

	Vector2d bl_rot = m_Rotation * bl;
	Vector2d br_rot = m_Rotation * br;
	Vector2d tr_rot = m_Rotation * tr;
	Vector2d tl_rot = m_Rotation * tl;

	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0] + bl_rot[0], m_Position[1] + bl_rot[1]);
	glVertex2f(m_Position[0] + br_rot[0], m_Position[1] + br_rot[1]);
	glVertex2f(m_Position[0] + tr_rot[0], m_Position[1] + tr_rot[1]);
	glVertex2f(m_Position[0] + tl_rot[0], m_Position[1] + tl_rot[1]);
	glEnd();
}
