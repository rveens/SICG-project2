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

bool CollisionSolver::detectCollision(std::vector<RigidBody *> &rbodies)
{
	// dimension, list overlaping intervals
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
				// We known we are looking at an interval-start-thingy.
				// add it to the active intervals
				active_intervals.push(std::get<0>(tup));
			} else if (std::get<1>(tup) == INTVL_TYPE::Ei) {
				// We known we are looking at an interval-end-thingy.
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

bool CollisionSolver::checkWithinTolerance()
{
	if (overlapping_rbs.empty())
		return false;
	// for each dimension (e.g. x, y)
	for (auto &pair : overlapping_rbs) {
		// find all intervals for a rigid body

		std::vector<INTVL> &list = pair.second;
		// we know: 
		// list[0] = intvl 1 dim x
		// list[1] = intvl 2 dim x
		// list[2] = intvl 1 dim x
		// list[3] = intvl 2 dim x

		// find minimum between x intervals 
		double toCheckX = std::min(list[0].si - list[1].ei, list[1].si - list[0].ei);

		// find minimum between y intervals 
		double toCheckY = std::min(list[2].si - list[3].ei, list[3].si - list[2].ei);
		if (toCheckX <= m_Tolerance && toCheckY <= m_Tolerance) {
			return true;
		}
	}
	return false;
}


void CollisionSolver::getPointOfCollision(Integrator *integrator, std::vector<RigidBody *> &rbodies, double timeStep)
{
	if (detectCollision(rbodies)) {

		// 1 determine middle step
		// 2 
		int i = 2;
		double tc = timeStep/2;
		do {
			int divisor = std::pow(2, i++);
			if (detectCollision(rbodies)) {
				tc -= timeStep/divisor; 
			} else {
				tc += timeStep/divisor;
			}
			for (RigidBody *rb : rbodies) {
				rb->setState(rb->m_PreviousState);
				// 2 do half a time step
				integrator->integrate(rb, tc);
			}
		} while(!checkWithinTolerance());
		for (auto &intervals : overlapping_rbs) {
			printf("collision at:\n");
			printf("(%f, %f)\n", intervals.second[0].si, intervals.second[2].si);

			Collision c1;
			c1.a = std::get<0>(intervals.first);
			c1.b = std::get<1>(intervals.first);
		}
	}
}

bool CollisionSolver::narrowCheck(RigidBody *rb1, RigidBody *rb2)
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

		printf("edge %d\n", i);
		/* std::cout << "a: "<< std::endl; */
		/* std::cout << a << std::endl; */
		/* std::cout << "b: "<< std::endl; */
		/* std::cout << b << std::endl; */

		/* std::cout << "rb1_normal: "<< std::endl; */
		/* std::cout << edge_normal << std::endl; */

		/* std::cout << "---------"<< std::endl; */
		/* std::cout << "rb1_vertices:" << std::endl; */
		/* for (auto k : rb1_vertices) { */
		/* 	std::cout << k << std::endl; */
		/* } */
		/* std::cout << "rb2_vertices:" << std::endl; */
		/* for (auto k : rb2_vertices) { */
		/* 	std::cout << k << std::endl; */
		/* } */

		// check all vertices of rb2
		for (Vector2d v : rb2_vertices) {
			double dot = testEdge(v, a, b, edge_normal);
			printf("dot: %f\n", dot);
			if (dot < 0)
				ok = false;
		}
		if (ok)
			return true;
	}
	/* exit(0); */

	return false;
}

double CollisionSolver::testEdge(Vector2d v, Vector2d a, Vector2d b, Vector2d ab_normal)
{
	return (v - a).dot(ab_normal);
}


/* double CollisionSolver::cross2D(Vector2d v, Vector2d w) */
/* { */
/* 	return v[0]*w[1] -v[1]*w[0]; */
/* } */

/* bool CollisionSolver::vectorIntersect(Vector2d p, Vector2d r, Vector2d q, Vector2d s, Vector2d &intersectionPoint) */
/* { */
/* 	double t = cross2D((q-p), s/cross2D(r, s)); */
/* 	double u = cross2D((q-p), r/cross2D(r, s)); */

/* 	// case 1 colinear */
/* 	if (cross2D(r, s) == 0 && cross2D((q-p), r) == 0) { */

/* 	} */
/* 	// case 2 parallel and no intersection */
/* 	if (cross2D(r, s) == 0 && cross2D((q-p), r) != 0) { */
/* 		// parallel */
/* 		return false; */
/* 	} */
/* 	// case 3 intersection */
/* 	if (cross2D(r, s) != 0 && (t >= 0 && t <= 1) && (u >= 0 && u <= 1)) { */
/* 		// intersection at p + tr OR q + us */
/* 		intersectionPoint = p + t*r; */
/* 		return true; */
/* 	} */
/* 	// case 4 Not parallel, no intersection */
/* 	return false; */
/* } */
