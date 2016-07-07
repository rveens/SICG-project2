#pragma once

#include "Contact.h"
#include "Integrator.h"

#include <vector>
#include <map>


class CollisionSolver
{
public:
	CollisionSolver();
	virtual ~CollisionSolver();

	/* functions */
	bool detectCollision(std::shared_ptr<RigidBody> rb1, std::shared_ptr<RigidBody> rb2);
	bool findContactPoints(std::shared_ptr<RigidBody> rb1, std::shared_ptr<RigidBody> rb2);
	void collisionResponse();


	/* public variables */
	std::vector<Contact> m_Contacts;

private:
	const double m_tolerance = 0.001;
	const double m_colliding_threshold = 0.1;


	/* helper functions */

	// used for detectCollision
	double projectOnEdgeNormal(Vector2d &v, Vector2d &a, Vector2d &ab_normal);
	bool SATIntervalTest(Vector2d &edgeNorm, Vector2d &a, std::vector<Vector2d> rb1_vertices, std::vector<Vector2d> rb2_vertices);
	bool vertexOnEdge(Vector2d &vert, std::tuple<Vector2d, Vector2d> &edge);

	// used for findAllCollisions
	Vector2d pointVelocity(std::shared_ptr<RigidBody> rb, Vector2d &point);
	bool colliding(Contact &c);
	void applyCollision(Contact &c);
						
	// 2D cross product
	double cross2D(Vector2d &a, Vector2d &b);
	
};
