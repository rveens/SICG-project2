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

	bool detectCollision(std::vector<RigidBody *> &rbodies);
	void getPointOfCollision(Integrator *integrator, std::vector<RigidBody *> &rbodies, double timeStep);

	std::map<std::tuple<RigidBody *, RigidBody *>, std::vector<INTVL>> overlapping_rbs;
	std::vector<Collision> m_Collisions;
	bool narrowCheck(RigidBody *rb1, RigidBody *rb2);

	std::vector<Vector2d> findContactPoints(RigidBody *rb1, RigidBody *rb2);
	void createCollisionObjects();

private:
	const double m_Tolerance = 0.5;

	bool checkWithinTolerance();

	double testEdge(Vector2d &v, Vector2d &a, Vector2d &b, Vector2d &ab_normal);


	bool vectorIntersect(Vector2d &p, Vector2d &r, Vector2d &q, Vector2d &s, Vector2d &intersectionPoint);
	double cross2D(Vector2d &a, Vector2d &b);
	int isVertexOfRb(Vector2d &intersection, RigidBody *rb, double epsilon);
};
