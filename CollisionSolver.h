#pragma once

#include "Collision.h"

#include <vector>
#include <map>


struct INTVL {
	double si = 0.0;
	double ei = 0.0;
	RigidBody *rb = 0;
	RigidBody *rb_other = 0;
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

	std::map<std::tuple<RigidBody *, RigidBody *>, std::vector<INTVL>> overlapping_rbs;
	std::vector<Collision> m_Collisions;

private:
	const double m_Tolerance = 0.5;

	// list of collisions
};
