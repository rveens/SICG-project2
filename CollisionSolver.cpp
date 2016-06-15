#include "CollisionSolver.h"

#include <map>
#include <stack>

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
