#pragma once

#include "Contact.h"
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
	bool detectCollision(RigidBody *rb1, RigidBody *rb2);
	void findContactPoints(RigidBody *rb1, RigidBody *rb2);

	/* public variables */
	std::vector<Contact> m_Contacts; // todo

private:
	const double m_tolerance = 0.001;

	/* helper functions */

	// used for detectCollision
	double projectOnEdgeNormal(Vector2d &v, Vector2d &a, Vector2d &ab_normal);
	bool SATIntervalTest(Vector2d &edgeNorm, Vector2d &a, std::vector<Vector2d> rb1_vertices, std::vector<Vector2d> rb2_vertices);
	bool vertexOnEdge(Vector2d &vert, std::tuple<Vector2d, Vector2d> &edge);
						
	
	// check if a vertex belongs to a rb within a certain epsilon
	int isVertexOfRb(Vector2d &vertex, RigidBody *rb, double epsilon);
	// check if two vectors (edges) intersect.
	bool vectorIntersect(Vector2d &p, Vector2d &r, Vector2d &q, Vector2d &s, Vector2d &intersectionPoint);
	// 2D cross product
	double cross2D(Vector2d &a, Vector2d &b);
	
};
