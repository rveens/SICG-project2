#include "RigidBody.h"

<<<<<<< HEAD
RigidBody::RigidBody(const Vector2d & ConstructPos, int mass, Matrix2d & Ibody,
			Matrix2d & IbodyInv, Matrix2d & rotation, Vector2d & linmom, Vector2d & angmom)
=======
RigidBody::RigidBody(const Vector2d & ConstructPos, int mass, Matrix2d Ibody, Matrix2d IbodyInv, 
		Matrix2d rotation)
>>>>>>> origin/master
	: m_ConstructPos(ConstructPos), m_Mass(mass), m_Ibody(Ibody), m_IbodyInv(IbodyInv), 
	m_Rotation(rotation), m_LinearMomentum(Vector2d::Zero()),
	m_AngularMomentum(Vector2d::Zero())
{

}

RigidBody::~RigidBody(void)
{

}

void RigidBody::reset()
{

}

void RigidBody::draw()
{

}

void RigidBody::setState(std::vector<double> state)
{
	int i = 0;

	m_Position[0] = state[i++];
	m_Position[1] = state[i++];

	m_Rotation(0, 0) = state[i++];
	m_Rotation(0, 1) = state[i++];
	m_Rotation(1, 0) = state[i++];
	m_Rotation(1, 1) = state[i++];

	m_LinearMomentum[0] = state[i++];
	m_LinearMomentum[1] = state[i++];

	m_AngularMomentum[0] = state[i++];
	m_AngularMomentum[1] = state[i++];

	/* compute some stuff */
	m_Velocity = m_LinearMomentum / m_Mass;
	m_Iinv = m_Rotation * m_IbodyInv * m_Rotation.transpose();
	m_Omega = m_Iinv * m_AngularMomentum;
}

std::vector<double> RigidBody::getState()
{
	std::vector<double> state;

	state.push_back(m_Position[0]);
	state.push_back(m_Position[1]);

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			state.push_back(m_Rotation(i, j));

	state.push_back(m_LinearMomentum[0]);
	state.push_back(m_LinearMomentum[1]);

	state.push_back(m_AngularMomentum[0]);
	state.push_back(m_AngularMomentum[1]);

	return state;
}

std::vector<double> RigidBody::derivEval()
{
	std::vector<double> der;

	der.push_back(m_Velocity[0]);
	der.push_back(m_Velocity[1]);

	Matrix2d Rdot = star(m_Omega) * m_Rotation;

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			der.push_back(Rdot(i, j));

	der.push_back(m_Force[0]);
	der.push_back(m_Force[1]);

	der.push_back(m_Torque[0]);
	der.push_back(m_Torque[1]);
	
	return der;
}

std::vector<double> RigidBody::derivEval(std::vector<double> input)
{
	std::vector<double> der;

	Vector2d tVelocity = Vector2d(input[6], input[7]) / m_Mass;
	der.push_back(tVelocity[0]);
	der.push_back(tVelocity[1]);

	Matrix2d tRot = Matrix2d();
	tRot(0, 0) = input[2];
	tRot(0, 1) = input[3];
	tRot(1, 0) = input[4];
	tRot(1, 1) = input[5];
	//FIXME
	/* Vector2d tOmega = tRot * m_IbodyInv * tRot.transpose(); */
	Vector2d tOmega;

	Matrix2d tRdot = star(tOmega) * tRot;

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			der.push_back(tRdot(i, j));

	der.push_back(input[6]);
	der.push_back(input[7]);

	der.push_back(input[8]);
	der.push_back(input[9]);

	return der;
}

Matrix2d RigidBody::star(Vector2d & a)
{
	Matrix2d out;

	out.setZero();

	out(0, 1) = -a[2];
	out(0, 2) = a[1];
	out(1, 0) = a[2];
	out(1, 2) = -a[0];
	out(2, 0) = -a[1];
	out(2, 1) = a[0];

	return out;
}
