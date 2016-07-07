#pragma once

#include "RigidBody.h"

#include <vector>
#include <tuple>
#include "Eigen/Dense"

using namespace Eigen;


class RigidBodyRectangle : public RigidBody
{
public:
	RigidBodyRectangle(const Vector2d & ConstructPos, Vector2d & size, int mass,
			Matrix2d & rotation);
	virtual ~RigidBodyRectangle(void);

	virtual void draw(int N);
	virtual std::vector<double> computeAABB();
	virtual std::vector<int> computeAABBcellAligned(int N);
	virtual std::vector<Vector2d> getVertices();

	virtual std::vector<std::tuple<Vector2d, Vector2d>> getEdges();
	virtual std::vector<Vector2d> getEdgeNormals();
	virtual void voxelize(int N);
	virtual std::vector<Vector2i> getBoundaryCells(int N, int *solid);
	virtual std::set<std::array<int, 2>> getSurroundingCells(int N, int *solid);


	// drawing functions
	virtual void drawbb();
	virtual void drawbbCells(int N);
	virtual void drawbbCellsOccupied(int N);
	virtual void drawBoundaryCells(int N, int *solid);
	virtual void drawEdgeNormals();


protected:
	Vector2d m_Size;

	bool checkIfPointInRectangle(Vector2d &point, int N);
};
