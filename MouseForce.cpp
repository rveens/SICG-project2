#include "MouseForce.h"
#include <GL/glut.h>

MouseForce::MouseForce(std::shared_ptr<RigidBody> rb, Vector2d &mousePos, double strength, bool mouse_dragged) :
	m_rb(rb), m_mousePos(mousePos), m_strength(strength), m_mouse_dragged(mouse_dragged) {}

MouseForce::~MouseForce()
{

}

void MouseForce::draw()
{
	if (m_mouse_dragged) {
		glLineWidth(3);
		glBegin(GL_LINES);
		glColor3f(0.6, 0.7, 0.8);
		glVertex2f(m_rb->m_Position[0], m_rb->m_Position[1]);
		glColor3f(0.6, 0.7, 0.8);
		glVertex2f(m_mousePos[0], m_mousePos[1]);
		glEnd();
	}
}

void MouseForce::calculateForce()
{
	if (m_mouse_dragged) {
		Vector2d l = m_mousePos - m_rb->m_Position;
		m_rb->m_Force = m_strength * l;
	}
}
