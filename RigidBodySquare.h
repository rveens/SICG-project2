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

	virtual void draw();
	virtual std::array<double, 4> computeAABB();
	virtual std::vector<Vector2d> getVertices();
	virtual std::vector<std::tuple<Vector2d, Vector2d>> getEdges();
	virtual std::vector<Vector2d> getEdgeNormals();

private:
	Vector2d m_Size;
};
