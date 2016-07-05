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

bool CollisionSolver::detectCollisionBroad(std::vector<RigidBody *> &rbodies)
{
	// dimension, list overlapping intervals
	std::vector<std::vector<INTVL>> intervals_overlapping;

	overlapping_rbs.clear();

	// for each dimension
	for (int i = 0; i < 2; i++) {
		std::vector<std::tuple<double, INTVL_TYPE>> list;
		std::map<double, INTVL> intervals;

		for (RigidBody *rb : rbodies) {
			// store si and ei into a list
			auto coords = rb->computeAABB();
			list.push_back(std::make_tuple(coords[i], INTVL_TYPE::Si));
			list.push_back(std::make_tuple(coords[i+2], INTVL_TYPE::Ei));
			struct INTVL itv;
			itv.rb = rb;
			itv.si = coords[i];
			itv.ei = coords[i+2];
			itv.overlap = false;
			itv.dimension = i;
			intervals[coords[i]] = itv;
		}

		std::stack<double> active_intervals;
		std::sort(list.begin(), list.end(), [](std::tuple<double, INTVL_TYPE> &t1, std::tuple<double, INTVL_TYPE> &t2) {
				return std::get<0>(t1) < std::get<0>(t2);
		});
		for (auto &tup : list) {
			if (std::get<1>(tup) == INTVL_TYPE::Si) {
				// We know we are looking at an interval-start-thingy.
				// add it to the active intervals
				active_intervals.push(std::get<0>(tup));
			} else if (std::get<1>(tup) == INTVL_TYPE::Ei) {
				// We know we are looking at an interval-end-thingy.
				//
				// Does the top interval-start-thingy on the 
				// stack fit with the interval-end-thingy ?

				double expectedEi = intervals[active_intervals.top()].ei;
				if (expectedEi == std::get<0>(tup)) {
					// no collision! pop
					active_intervals.pop();
				} else {
					// collision
					intervals[active_intervals.top()].overlap = true;
					// find other interval
					for (auto &pair : intervals) {
						if (pair.second.ei == std::get<0>(tup))
							pair.second.overlap = true;
					}
				}
			}
		}
		intervals_overlapping.push_back(std::vector<INTVL>());
		// add overlapping intervals to a list
		for (auto pair : intervals) {
			// note: pair.second is a ITVL type
			if (pair.second.overlap) {
				intervals_overlapping[i].push_back(pair.second);
			}
		}
	}

	// check if there there is a collision:
	for (auto &list : intervals_overlapping) {
		if (list.size() == 2) {
			list[0].rb_other = list[1].rb;
			list[1].rb_other = list[0].rb;

			overlapping_rbs[std::make_tuple(list[0].rb, list[1].rb)].push_back(list[0]);
			overlapping_rbs[std::make_tuple(list[0].rb, list[1].rb)].push_back(list[1]);

/* 			collision_intervals.push_back(std::make_tuple(list[0], list[1])); */

/* 			INTVL &intvl_a = pair.second[0]; */
/* 			INTVL &intvl_b = pair.second[1]; */

/* 			Collision collision; */
/* 			collision.a = intvl_a.rb; */
/* 			collision.b = intvl_b.rb; */
/* 			/1* colision. *1/ */
/* 			m_Collisions.push_back(collision); */
		}
	}

	return !overlapping_rbs.empty();
}

bool CollisionSolver::detectCollisionNarrow(RigidBody *rb1, RigidBody *rb2)
{
	auto rb1_edges = rb1->getEdges();
	auto rb1_normals = rb1->getEdgeNormals();
	auto rb2_vertices = rb2->getVertices();

	auto rb1_vertices = rb1->getVertices();


	// for each edge of rb1
	for (int i = 0; i < rb1_edges.size(); i++) {
		bool ok = true;
		auto tuple = rb1_edges[i];
		Vector2d a = std::get<0>(tuple);
		Vector2d b = std::get<0>(tuple) + std::get<1>(tuple);
		Vector2d edge_normal = rb1_normals[i];

		/* printf("edge %d\n", i); */
		/* std::cout << "a: "<< std::endl; */
		/* std::cout << a << std::endl; */
		/* std::cout << "b: "<< std::endl; */
		/* std::cout << b << std::endl; */

		/* std::cout << "rb1_normal: "<< std::endl; */
		/* std::cout << edge_normal << std::endl; */


		// check all vertices of rb2
		for (Vector2d v : rb2_vertices) {
			double dot = SATtest(v, a, b, edge_normal);
			/* printf("dot: %f\n", dot); */
			if (dot < 0)
				ok = false;
		}
		if (ok)
			return false;
	}

	return true;
}

double CollisionSolver::SATtest(Vector2d &v, Vector2d &a, Vector2d &b, Vector2d &ab_normal)
{
	return (v - a).dot(ab_normal);
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
