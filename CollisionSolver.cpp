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
		Vector2d b = std::get<1>(tuple);
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
		Vector2d b = std::get<1>(tuple);
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

std::vector<Vector2d> CollisionSolver::findContactPoints(RigidBody *rb1, RigidBody *rb2)
{
	m_Collisions.clear();

	auto rb1_edges = rb1->getEdges();
	auto rb2_edges = rb2->getEdges();
	auto rb1_edgeNormals = rb1->getEdgeNormals();
	auto rb2_edgeNormals = rb2->getEdgeNormals();

	std::vector<Vector2d> intersections;

	// for each edge of r1, check intersection with every edge of rb2
	for (int i = 0; i < rb1_edges.size(); i ++) {
		auto tuplerb1 = rb1_edges[i];
		for (int j = 0; j < rb2_edges.size(); j ++) {
			auto tuplerb2 = rb2_edges[j];
			Vector2d p = std::get<0>(tuplerb1);
			Vector2d r = std::get<0>(tuplerb1) + std::get<1>(tuplerb1);

			Vector2d q = std::get<0>(tuplerb2);
			Vector2d s = std::get<0>(tuplerb2) + std::get<1>(tuplerb2);

			Vector2d output = Vector2d::Zero();
			vectorIntersect(p, r, q, s, output);
			if (!output.isZero()) {
				intersections.push_back(output);
				int idx = isVertexOfRb(output, rb1, 0.01);
				if (idx != -1) {
					printf("intersection is vertex %d of %d\n", idx, rb1);
					/* exit(0); */
				}
				idx = isVertexOfRb(output, rb2, 0.01);
				if (idx != -1) {
					printf("intersection is vertex %d of %d\n", idx, rb2);
					/* exit(0); */
				}

				Collision col;
				col.a = rb1;
				col.b = rb2;
				col.n = rb1_edgeNormals[i];
				col.p = output;
				Vector2d aEdge = std::get<1>(rb1_edges[i]);
				aEdge.normalize();
				col.ea = aEdge;
				Vector2d bEdge = std::get<1>(rb2_edges[j]);
				bEdge.normalize();
				col.eb = bEdge;
				col.vf = false;
			}
		}
	}
	std::cout << "intersections: "<< std::endl;
	for (auto v : intersections) {
		std::cout << v[0] << ", " << v[1] << std::endl;
		/* std::cout << "---------"<< std::endl; */
		/* std::cout << "rb1_vertices:" << std::endl; */
		/* for (auto k : rb1->getVertices()) { */
		/* 	std::cout << k << std::endl; */
		/* } */
		/* std::cout << "rb2_vertices:" << std::endl; */
		/* for (auto k : rb2->getVertices()) { */
		/* 	std::cout << k << std::endl; */
		/* } */
		/* exit(0); */
	}


	return intersections;
}


int CollisionSolver::isVertexOfRb(Vector2d &intersection, RigidBody *rb, double epsilon = 0.0001)
{
	auto rbVertices = rb->getVertices();

	for (int i = 0; i < rbVertices.size(); ++i) {
		Vector2d v = rbVertices[i];

		if (v[0] + epsilon >= intersection[0] && v[0] - epsilon <= intersection[0]) {
			if (v[1] + epsilon >= intersection[1] && v[1] - epsilon <= intersection[1]) {
				return i;
			}
		}
	}

	return -1;
}
