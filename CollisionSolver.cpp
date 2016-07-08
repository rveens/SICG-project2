#include "CollisionSolver.h"

#include <memory>
#include <map>
#include <stack>
#include <iostream>

CollisionSolver::CollisionSolver()
{

}

CollisionSolver::~CollisionSolver()
{

}

bool CollisionSolver::detectCollision(std::shared_ptr<RigidBody> rb1, std::shared_ptr<RigidBody> rb2)
{
	auto rb1_edges = rb1->getEdges();
	auto rb2_edges = rb2->getEdges();

	auto rb1_normals = rb1->getEdgeNormals();
	auto rb2_normals = rb2->getEdgeNormals();

	auto rb1_vertices = rb1->getVertices();
	auto rb2_vertices = rb2->getVertices();

	int i = 0;
	for (auto tuple : rb1_edges) {
		Vector2d a = std::get<0>(tuple);
		Vector2d edgeNormal = rb1_normals[i];

		// did we find a gap? if we find no gap for all of them we have a collision
		// if we find a gap, then there is no collision
		if (SATIntervalTest(edgeNormal, a, rb1_vertices, rb2_vertices)) {
			return false;
		}

		i++;
	}
	i = 0;
	for (auto tuple : rb2_edges) {
		Vector2d a = std::get<0>(tuple);
		Vector2d edgeNormal = rb2_normals[i];

		// did we find a gap? if we find no gap for all of them we have a collision
		// if we find a gap, then there is no collision
		if (SATIntervalTest(edgeNormal, a, rb2_vertices, rb1_vertices)) {
			return false;
		}

		i++;
	}

	return true;
}

bool CollisionSolver::findContactPoints(std::shared_ptr<RigidBody> rb1, std::shared_ptr<RigidBody> rb2)
{
	m_Contacts.clear();
	auto rb1_edges = rb1->getEdges();
	auto rb2_edges = rb2->getEdges();

	auto rb1_vertices = rb1->getVertices();
	auto rb2_vertices = rb2->getVertices();

	auto rb1_edgeNormals = rb1->getEdgeNormals();
	auto rb2_edgeNormals = rb2->getEdgeNormals();

	// check for each vertex of rb1, if the vertex is in contact with an edges of rb2.
	for (Vector2d &vert : rb1_vertices) {
		int i = 0;
		for (std::tuple<Vector2d, Vector2d> &edge : rb2_edges) {
			if (vertexOnEdge(vert, edge)) {
				//printf("Vertex-edge collision!\n");
				Contact c;
				c.a = rb1;
				c.b = rb2;
				c.p = vert;
				c.n = rb2_edgeNormals[i];
				//c.ea = { 0, 0 }; // todo
				//c.eb = { 0, 0 };
				//c.vf = true;
				c.edge = edge;
				m_Contacts.push_back(c);
			}
			i++;
		}
	}

	// check for each vertex of rb2, if the vertex is in contact with an edge of rb1.
	for (Vector2d &vert : rb2_vertices) {
		int i = 0;
		for (std::tuple<Vector2d, Vector2d> &edge : rb1_edges) {
			if (vertexOnEdge(vert, edge)) {
				//printf("Vertex-edge collision!\n");
				Contact c;
				c.a = rb2;
				c.b = rb1;
				c.p = vert;
				c.n = rb1_edgeNormals[i];
				//c.ea = { 0, 0 }; // todo
				//c.eb = { 0, 0 };
				//c.vf = true;
				c.edge = edge;
				m_Contacts.push_back(c);
			}
			i++;
		}
	}

	return !m_Contacts.empty();
}

void CollisionSolver::collisionResponse()
{
	bool had_collision;
	do {
		had_collision = false;

		for (Contact &c : m_Contacts) {
			if (colliding(c)) {
				applyCollision(c);
				had_collision = true;
			}
		}
	} while (had_collision == true);
}

double CollisionSolver::projectOnEdgeNormal(Vector2d &v, Vector2d &a, Vector2d &ab_normal)
{
	return (v - a).dot(ab_normal);
}

bool CollisionSolver::SATIntervalTest(Vector2d &edgeNorm, Vector2d &a, std::vector<Vector2d> rb1_vertices, std::vector<Vector2d> rb2_vertices)
{
	// project each vertex on the given edge normal for rb1 and find min and max
	std::vector<double> rb1_projections;
	for (Vector2d v : rb1_vertices) {
		rb1_projections.push_back(projectOnEdgeNormal(v, a, edgeNorm));
	}
	// get min and max
	auto minmax = std::minmax_element(begin(rb1_projections), end(rb1_projections), 
		[](auto one, auto other) {
		return one < other;
	});
	double rb1_min = *minmax.first;
	double rb1_max = *minmax.second;


	// project each vertex on the given edge normal for rb1 and find min and max
	std::vector<double> rb2_projections;
	for (Vector2d v : rb2_vertices) {
		rb2_projections.push_back(projectOnEdgeNormal(v, a, edgeNorm));
	}
	// get min and max
	auto minmax2 = std::minmax_element(begin(rb2_projections), end(rb2_projections),
		[](auto one, auto other) {
		return one < other;
	});
	double rb2_min = *minmax2.first;
	double rb2_max = *minmax2.second;

	// finally, check if our two computed intervals overlap, if so we have a possible collision
	if ( (rb1_min < rb2_max &&rb2_min < rb1_max) ||		// rb1 overlaps rb2 partially
		(rb1_min < rb2_min && rb1_max > rb2_max) ||		// rb2 is inside rb1
		(rb2_min < rb1_min && rb2_max > rb1_max) ) {	// rb1 is inside rb2
		return false;
	}
	else
		return true;
}

bool CollisionSolver::vertexOnEdge(Vector2d &vert, std::tuple<Vector2d, Vector2d> &edge)
{
	// check if the given vertex is on the given edge. If so, return true.

	// test if point is on the line. http://stackoverflow.com/questions/7050186/find-if-point-lays-on-line-segment
	Vector2d a = std::get<0>(edge);
	Vector2d b = std::get<0>(edge) + std::get<1>(edge);

	double AB = std::sqrt(pow(b[0] - a[0], 2) + pow(b[1] - a[1], 2));
	double AP = std::sqrt(pow(vert[0] - a[0], 2) + pow(vert[1] - a[1], 2));
	double PB = std::sqrt(pow(b[0] - vert[0], 2) + pow(b[1] - vert[1], 2));

	if (AB <= AP + PB + m_tolerance && AB >= AP + PB - m_tolerance) {
		return true;
	}
	else
		return false;
}

Vector2d CollisionSolver::pointVelocity(std::shared_ptr<RigidBody> rb, Vector2d &point)
{
	double omega = rb->getOmega();
	double x = point[0] - rb->m_Position[0];
	double y = point[1] - rb->m_Position[1];

	return rb->getVelocity() + Vector2d(-omega*y, omega*x);
}

bool CollisionSolver::colliding(Contact &c)
{
	Vector2d padot = pointVelocity(c.a, c.p);
	Vector2d pbdot = pointVelocity(c.b, c.p);
	double vrel = c.n.dot(padot - pbdot);

	// tolerances misschien anders maken hier, want dat is in het voorbeeld ook
	if (vrel > m_colliding_threshold) // moving away
		return false;
	// we now know vrel < m_colliding_threshold
	if (vrel > -m_colliding_threshold) // resting contact
		return false;
	else
		return true;
}

void CollisionSolver::applyCollision(Contact &c)
{
	Vector3d padot;
	{
		Vector2d temp = pointVelocity(c.a, c.p);
		padot= Vector3d(temp[0], temp[1], 0);
	}
	Vector3d pbdot;
	{
		Vector2d temp = pointVelocity(c.b, c.p);
		pbdot = Vector3d(temp[0], temp[1], 0);
	}
	Vector3d n = Vector3d(c.n[0], c.n[1], 0);
	Vector3d ra = Vector3d(c.p[0] - c.a->m_Position[0], c.p[1] - c.a->m_Position[1], 0); ;
	Vector3d rb = Vector3d(c.p[0] - c.b->m_Position[0], c.p[1] - c.b->m_Position[1], 0); ;
	double vrel = n.dot(padot - pbdot);
	double numerator = -(1 + m_tolerance) * vrel;

	double aInv = c.a->getIinv();
	double bInv = c.b->getIinv();

	double term1 = 1 / c.a->m_Mass;
	double term2 = 1 / c.b->m_Mass;
	
	double term3 = n.dot(aInv * (ra.cross(n)).cross(ra)); // gaat dit goed?
	double term4 = n.dot(bInv * (rb.cross(n)).cross(rb)); // gaat dit goed?

	double j = numerator / (term1 + term2 + term3 + term4);
	Vector3d force = j*n;

	// apply impulse to bodies
	double collision_strength = 3;
	c.a->m_LinearMomentum[0] += force[0] * collision_strength;
	c.a->m_LinearMomentum[1] += force[1] * collision_strength;

	c.b->m_LinearMomentum[0] -= force[0] * collision_strength;
	c.b->m_LinearMomentum[1] -= force[1] * collision_strength;


	c.a->m_AngularMomentum += ra.cross(force)[2];

	c.b->m_AngularMomentum -= rb.cross(force)[2];


	// compute aux vars
	c.a->computeAuxVariables();
	c.b->computeAuxVariables();
}


double CollisionSolver::cross2D(Vector2d &v, Vector2d &w)
{
	return v[0]*w[1] -v[1]*w[0];
}