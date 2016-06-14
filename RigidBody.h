#pragma once

#include <vector>

#include "Eigen/Dense"

using namespace Eigen;

class RigidBody
{
public:
	RigidBody(const Vector2d & ConstructPos, int mass, Matrix2d & rotation);
	virtual ~RigidBody(void);

	/* public functions: */
	virtual void draw();

	void reset();
	void setState(VectorXd state);
	VectorXd getState();
	VectorXd derivEval();
	VectorXd derivEval(VectorXd input);

	/* public variables: */

	/* constant quantities */
	const Vector2d m_ConstructPos;
	int m_Mass;
	Matrix2d m_Ibody,
		 m_IbodyInv;

	/* state variables */
	Vector2d m_Position;		// x
	Matrix2d m_Rotation;		// R
	Vector2d m_LinearMomentum; 	// P
	Vector2d m_AngularMomentum;	// L

	/* derived quantities */
	Matrix2d m_Iinv;
	Vector2d m_Velocity;
	Vector2d m_Omega;

	/* computed quantities */
	Vector2d m_Force;
	Vector2d m_Torque;
private:
	// helper function, compute a* matrix

	Matrix2d star(Vector2d & a);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
