#include "RigidBody.h"
#include <iostream>
#include <GL/glut.h>


RigidBody::RigidBody(const Vector2d & ConstructPos, int mass, Matrix2d & rotation)
	: m_ConstructPos(ConstructPos), m_Mass(mass),
	m_Position(ConstructPos), m_Rotation(rotation), m_LinearMomentum(Vector2d(0, 0)),
	m_AngularMomentum(0),
	m_Iinv(0), m_Velocity(Vector2d(0, 0)), m_Omega(0), 
	m_Force(Vector2d(0, 0)), m_Torque(0.0)
{
}

RigidBody::~RigidBody(void)
{

}

void RigidBody::reset()
{

}

void RigidBody::setState(const VectorXd &state)
{
	int i = 0;

	m_Position[0] = state(i++);
	m_Position[1] = state(i++);

	m_Rotation(0, 0) = state(i++);
	m_Rotation(0, 1) = state(i++);
	m_Rotation(1, 0) = state(i++);
	m_Rotation(1, 1) = state(i++);

	m_LinearMomentum[0] = state(i++);
	m_LinearMomentum[1] = state(i++);

	m_AngularMomentum = state(i++);

	/* compute some stuff */
	m_Velocity = m_LinearMomentum / m_Mass;
	//m_Iinv = m_Rotation * m_IbodyInv * m_Rotation.transpose();
	m_Iinv = m_IbodyInv;
	m_Omega = m_Iinv * m_AngularMomentum;
}

VectorXd RigidBody::getState()
{
	VectorXd state = VectorXd::Zero(9);
	int i = 0;

	state(i++) = m_Position[0];
	state(i++) = m_Position[1];

	for (int k = 0; k < 2; k++)
		for (int j = 0; j < 2; j++)
			state(i++) = m_Rotation(k, j);

	state(i++) = m_LinearMomentum[0];
	state(i++) = m_LinearMomentum[1];

	state(i++) = m_AngularMomentum;

	return state;
}

VectorXd RigidBody::derivEval()
{
	VectorXd der = VectorXd::Zero(9);

	int i = 0;

	der(i++) = m_Velocity[0];
	der(i++) = m_Velocity[1];

	// "3D case":
	//Matrix2d Rdot = star(m_Omega) * m_Rotation;
	
	// 2D case:
	Matrix2d Rdot;
	Rdot << -m_Rotation(1,0), -m_Rotation(1,1),
		     m_Rotation(0,0),  m_Rotation(0,1);
	Rdot *= m_Omega;

	for (int k = 0; k < 2; k++)
		for (int j = 0; j < 2; j++)
			der(i++) = Rdot(k, j);

	der(i++) = m_Force[0];
	der(i++) = m_Force[1];

	der(i++) = m_Torque;
	
	return der;
}

VectorXd RigidBody::derivEval(const VectorXd &input)
{
	VectorXd der(9);
	int i = 0;

	der.setZero();
	
	Vector2d tVelocity = Vector2d(input[6], input[7]) / m_Mass;
	der(i++) = tVelocity[0];
	der(i++) = tVelocity[1];

	Matrix2d tRot = Matrix2d::Zero();
	tRot(0, 0) = input[2];
	tRot(0, 1) = input[3];
	tRot(1, 0) = input[4];
	tRot(1, 1) = input[5];
	double tL = input[8];
	double tOmega = m_IbodyInv * tL;

	// "3D case":
	//Matrix2d tRdot = star(tOmega) * tRot;

	// 2D case:
	Matrix2d tRdot;
	tRdot << -tRot(1,0), -tRot(1,1),
		      tRot(0,0),  tRot(0,1);
	tRdot *= tOmega;

	for (int k = 0; k < 2; k++)
		for (int j = 0; j < 2; j++)
			der(i++) = tRdot(k, j);

	der[i++] = input[6];
	der[i++] = input[7];

	der[i++] = input[8];

	return der;
}

Matrix2d RigidBody::star(Vector2d & a)
{
	/* std::cout << "a (omega)" << std::endl; */
	/* std::cout << a << std::endl; */

	Matrix2d out = Matrix2d::Zero();

	if (a.isZero())
		return out;

	out(0, 0) = 0;
	out(0, 1) = a[0];
	out(1, 0) = -a[1];
	out(1, 1) = 0;

	return out;
}


Vector2d RigidBody::getVelocity()
{
	return m_Velocity;
}

double RigidBody::getOmega()
{
	return m_Omega;
}

double RigidBody::getIinv()
{
	return m_Iinv;
}

void RigidBody::computeAuxVariables()
{
	m_Velocity = m_LinearMomentum / m_Mass;
	m_Omega = m_Iinv * m_AngularMomentum;
}