#include "RigidBodyWall.h"

#include <GL/glut.h>

RigidBodyWall::~RigidBodyWall()
{

}

void RigidBodyWall::draw(int N)
{
	Vector2d bl = Vector2d(-m_Size[0] / 2, -m_Size[1] / 2);
	Vector2d br = Vector2d(+m_Size[0] / 2, -m_Size[1] / 2);
	Vector2d tr = Vector2d(+m_Size[0] / 2, +m_Size[1] / 2);
	Vector2d tl = Vector2d(-m_Size[0] / 2, +m_Size[1] / 2);

	Vector2d bl_rot = m_Rotation * bl;
	Vector2d br_rot = m_Rotation * br;
	Vector2d tr_rot = m_Rotation * tr;
	Vector2d tl_rot = m_Rotation * tl;

	glColor3f(0.49411764705882352941f, 0.51372549019607843137f, 0.52372549019607843137f);
	glLineWidth(1);
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0] + bl_rot[0], m_Position[1] + bl_rot[1]);
	glVertex2f(m_Position[0] + br_rot[0], m_Position[1] + br_rot[1]);
	glVertex2f(m_Position[0] + tr_rot[0], m_Position[1] + tr_rot[1]);
	glVertex2f(m_Position[0] + tl_rot[0], m_Position[1] + tl_rot[1]);
	glEnd();
}

void RigidBodyWall::setState(const VectorXd &state)
{
	// do nothing on purpose
}

VectorXd RigidBodyWall::derivEval()
{
	// do nothing on purpose
	return getState();
}

VectorXd RigidBodyWall::derivEval(const VectorXd &input)
{
	// do nothing on purpose
	return getState();
}