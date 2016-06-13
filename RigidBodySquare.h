#pragma once

#include "RigidBody.h"

#include <vector>
#include "Eigen/Dense"

using namespace Eigen;


class RigidBodySquare : public RigidBody
{
public:
	RigidBodySquare(const Vector2d & ConstructPos, Vector2d size, int mass,
			Matrix2d rotation);
	virtual ~RigidBodySquare(void);

	void draw();

private:
	Vector2d m_Size;
};
