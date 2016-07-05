#pragma once

#include <vector>

#include "Eigen/Dense"

#include "SolverInterface.h"

using namespace Eigen;

class RigidBody : public SolverInterface
{
public:
	RigidBody(const Vector2d & ConstructPos, int mass, Matrix2d & rotation);
	virtual ~RigidBody(void);

	/* public functions: */
	virtual void draw(int N) = 0;

	void reset();
	void setState(const VectorXd &state);
	VectorXd getState();
	VectorXd derivEval();
	VectorXd derivEval(const VectorXd &input);
	virtual std::vector<double> computeAABB() = 0;
	virtual std::vector<int> computeAABBcellAligned(int N) = 0;
	virtual std::vector<Vector2d> getVertices() = 0;
	virtual std::vector<std::tuple<Vector2d, Vector2d>> getEdges() = 0;
	virtual std::vector<Vector2d> getEdgeNormals() = 0;
	virtual void voxelize(int N) = 0;
	virtual std::vector<Vector2i> getBoundaryCells(int N, int *solid) = 0;

	// drawing functions
	virtual void drawbb() { };
	virtual void drawbbCells(int N) { };
	virtual void drawbbCellsOccupied(int N) { };
	virtual void drawBoundaryCells(int N, int *) { };
	virtual void drawEdgeNormals() { };


	/* public variables: */
	std::vector<Vector2i> gridIndicesOccupied;
	std::vector<Vector2i> gridIndicesOccupiedPreviously;
	std::vector<Vector2i> gridIndicesCloseToBoundary;

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


	/* computed quantities */
	Vector2d m_Force;
	double m_Torque;

	/* previous state for collision detection */
	VectorXd m_PreviousState;
private:
	// helper function, compute a* matrix

	Matrix2d star(Vector2d & a);


	/* derived quantities */
	Matrix2d m_Iinv;
	Vector2d m_Velocity;
	Vector2d m_Omega;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
