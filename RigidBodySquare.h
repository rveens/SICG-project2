#pragma once

#include "RigidBody.h"

#include <vector>
#include <tuple>
#include "Eigen/Dense"

using namespace Eigen;


class RigidBodySquare : public RigidBody
{
public:
	RigidBodySquare(const Vector2d & ConstructPos, Vector2d & size, int mass,
			Matrix2d & rotation);
	virtual ~RigidBodySquare(void);

	virtual void draw(int N);
	virtual std::vector<double> computeAABB();
	virtual std::vector<int> computeAABBcellAligned(int N);
	virtual std::vector<Vector2d> getVertices();

	virtual std::vector<std::tuple<Vector2d, Vector2d>> getEdges();
	virtual std::vector<Vector2d> getEdgeNormals();
	virtual void voxelize(int N);
	virtual std::vector<Vector2i> getBoundaryCells(int N, int *solid);


	// drawing functions
	virtual void drawbb();
	virtual void drawbbCells(int N);
	virtual void drawbbCellsOccupied(int N);
	virtual void drawBoundaryCells(int N, int *solid);
	virtual void drawEdgeNormals();


private:
	Vector2d m_Size;

	bool checkIfPointInSquare(Vector2d &point);
};
