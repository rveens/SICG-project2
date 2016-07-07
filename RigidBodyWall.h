#pragma once

#include "RigidBodyRectangle.h"

class RigidBodyWall: public RigidBodyRectangle
{
public:
	using RigidBodyRectangle::RigidBodyRectangle;
	~RigidBodyWall();

	void draw(int N) override;

	void setState(const VectorXd &state) override;
	VectorXd derivEval() override;
	VectorXd derivEval(const VectorXd &input) override;

	void voxelize(int N) override;
private:
	const Matrix2d m_constructRotation;
	bool hasVoxelized = false;
};