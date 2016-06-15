#pragma once

#include "Collision.h"

#include <vector>
#include <map>


struct INTVL {
	double si = 0.0;
	double ei = 0.0;
	RigidBody *rb;
	bool overlap = false;
	int dimension = 0;
};
enum INTVL_TYPE {
	Si,
	Ei
};

class CollisionSolver
{
public:
	CollisionSolver();
	virtual ~CollisionSolver();

	bool detectCollision(std::vector<RigidBody *> &rbodies);
	bool checkWithinTolerance();

private:
	std::vector<Collision> m_Collisions;
	std::map<RigidBody *, std::vector<INTVL>> collision_intervals;

	const double tolerance = 0.01;
};
