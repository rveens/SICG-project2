#include "CollisionSolver.h"

#include <map>
#include <stack>
#include <iostream>

CollisionSolver::CollisionSolver()
{

}

CollisionSolver::~CollisionSolver()
{

}

bool CollisionSolver::detectCollision(RigidBody *rb1, RigidBody *rb2)
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

double CollisionSolver::cross2D(Vector2d &v, Vector2d &w)
{
	return v[0]*w[1] -v[1]*w[0];
}

bool CollisionSolver::vectorIntersect(Vector2d &p, Vector2d &r, Vector2d &q, Vector2d &s, Vector2d &intersectionPoint)
{
	Vector2d qminp = q - p;
	// case 1 colinear
	if (cross2D(r, s) == 0 && cross2D(qminp, r) == 0) {
		if ( (0 <= (q - p).dot(r) && (q - p).dot(r) <= r.dot(r)) || 
			 (0 <= (p - q).dot(s) && (p - q).dot(s) <= s.dot(s)) )
			return true;

		return false;
	}
	// case 2 parallel and no intersection
	if (cross2D(r, s) == 0 && cross2D(qminp, r) != 0) {
		// parallel
		return false;
	}
	Vector2d t_second = s / cross2D(r, s);
	double t = cross2D(qminp, t_second);
	Vector2d u_second = r / cross2D(r, s);
	double u = cross2D(qminp, u_second);

	// case 3 intersection
	if (cross2D(r, s) != 0 && (t >= 0 && t <= 1) && (u >= 0 && u <= 1)) {
		// intersection at p + tr OR q + us
		intersectionPoint = p + t*r;
		return true;
	}
	// case 4 Not parallel, no intersection
	return false;
}

void CollisionSolver::findContactPoints(RigidBody *rb1, RigidBody *rb2)
{
	m_Contacts.clear();
	auto rb1_edges = rb1->getEdges();
	auto rb2_edges = rb2->getEdges();

	auto rb1_vertices = rb1->getVertices();
	auto rb2_vertices = rb2->getVertices();

	// check for each vertex of rb1, if the vertex is in contact with an edges of rb2.
	for (Vector2d &vert : rb1_vertices) {
		for (std::tuple<Vector2d, Vector2d> &edge : rb2_edges) {
			if (vertexOnEdge(vert, edge)) {
				printf("Vertex-edge collision!\n");
				Contact c;
				c.a = rb1;
				c.b = rb2;
				c.p = vert;
				c.n = { 0, 0 }; // todo
				c.ea = { 0, 0 }; // todo
				c.eb = { 0, 0 };
				c.vf = true;
				c.edge = edge;
				m_Contacts.push_back(c);
			}
		}
	}

	// check for each vertex of rb2, if the vertex is in contact with an edge of rb1.
	for (Vector2d &vert : rb2_vertices) {
		for (std::tuple<Vector2d, Vector2d> &edge : rb1_edges) {
			if (vertexOnEdge(vert, edge)) {
				printf("Vertex-edge collision!\n");
				Contact c;
				c.a = rb2;
				c.b = rb1;
				c.p = vert;
				c.n = { 0, 0 }; // todo
				c.ea = { 0, 0 }; // todo
				c.eb = { 0, 0 };
				c.vf = true;
				c.edge = edge;
				m_Contacts.push_back(c);
			}
		}
	}
}

bool CollisionSolver::vertexOnEdge(Vector2d &vert, std::tuple<Vector2d, Vector2d> &edge)
{
	// check if the given vertex is on the given edge. If so, return true.

	// test if point is on the line. http://stackoverflow.com/questions/7050186/find-if-point-lays-on-line-segment
	Vector2d a = std::get<0>(edge);
	Vector2d b = std::get<0>(edge) + std::get<1>(edge);

	double AB = std::sqrt( pow(b[0] - a[0], 2) + pow(b[1]-a[1], 2) );
	double AP = std::sqrt( pow(vert[0] - a[0], 2) + pow(vert[1] - a[1], 2) );
	double PB = std::sqrt( pow(b[0] - vert[0], 2) + pow(b[1] - vert[1], 2) );

	if (AB <= AP + PB + m_tolerance && AB >= AP + PB - m_tolerance) {
		return true;
	}
	else
		return false;

	/*Vector2d a = std::get<0>(edge);
	Vector2d b = std::get<0>(edge) + std::get<1>(edge);
	double lhs = (vert[0] - a[0]) / (b[0] - a[0]);
	double rhs = (vert[1] - a[1]) / (b[1] - a[1]);
	if (lhs <= rhs + m_tolerance && lhs >= rhs - m_tolerance) {
		return true;
	}
	else
		return false;*/
}

int CollisionSolver::isVertexOfRb(Vector2d &vertex, RigidBody *rb, double epsilon = 0.0001)
{
	auto rbVertices = rb->getVertices();

	for (int i = 0; i < rbVertices.size(); ++i) {
		Vector2d v = rbVertices[i];

		if (v[0] + epsilon >= vertex[0] && v[0] - epsilon <= vertex[0]) {
			if (v[1] + epsilon >= vertex[1] && v[1] - epsilon <= vertex[1]) {
				return i;
			}
		}
	}

	return -1;
}