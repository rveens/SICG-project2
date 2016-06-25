#pragma once

#include <vector>
//#include <array>

#include "Eigen/Dense"

using namespace Eigen;

class RigidBody
{
public:
	RigidBody(const Vector2d & ConstructPos, int mass, Matrix2d & rotation);
	virtual ~RigidBody(void);

	/* public functions: */
	virtual void draw() = 0;

	void reset();
	void setState(VectorXd state);
	VectorXd getState();
	VectorXd derivEval();
	VectorXd derivEval(VectorXd input);
	virtual std::vector<double> computeAABB() = 0;
	virtual std::vector<int> computeAABBcellAligned(int N) = 0;
	virtual std::vector<Vector2d> getVertices() = 0;
	virtual std::vector<std::tuple<Vector2d, Vector2d>> getEdges() = 0;
	virtual std::vector<Vector2d> getEdgeNormals() = 0;

	/* public variables: */

	/* drawing variables: */
	bool m_Drawbb = false;
	bool m_DrawbbCells = false;

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

	/* previous state for collision detection */
	VectorXd m_PreviousState;
private:
	// helper function, compute a* matrix

	Matrix2d star(Vector2d & a);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
