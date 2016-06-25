#pragma once

#include "Collision.h"
#include "Integrator.h"

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

	/* functions */
	bool detectCollisionBroad(std::vector<RigidBody *> &rbodies);
	bool detectCollisionNarrow(RigidBody *rb1, RigidBody *rb2);

	std::vector<Vector2d> findContactPoints(RigidBody *rb1, RigidBody *rb2);

	/* public variables */
	std::map<std::tuple<RigidBody *, RigidBody *>, std::vector<INTVL>> overlapping_rbs;
	std::vector<Collision> m_Collisions; // todo

private:
	
	/* helper functions */

	// check if two vectors (edges) intersect.
	bool vectorIntersect(Vector2d &p, Vector2d &r, Vector2d &q, Vector2d &s, Vector2d &intersectionPoint);
	// used for detectCollisionNarrow
	double SATtest(Vector2d &v, Vector2d &a, Vector2d &b, Vector2d &ab_normal);
	// 2D cross product
	double cross2D(Vector2d &a, Vector2d &b);
	// check if a vertex belongs to a rb within a certain epsilon
	int isVertexOfRb(Vector2d &intersection, RigidBody *rb, double epsilon);
};
