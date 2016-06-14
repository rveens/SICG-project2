#include "CollisionSolver.h"

#include <map>
#include <stack>

CollisionSolver::CollisionSolver()
{

}

CollisionSolver::~CollisionSolver()
{

}

void CollisionSolver::detectCollisions(std::vector<RigidBody *> &rbodies)
{
	enum INTVL_TYPE {
		Si,
		Ei
	};

	struct INTVL {
		double si = 0.0;
		double ei = 0.0;
		RigidBody *rb;
		bool overlap = false;
		int dimension = 0;
	};

	std::map<RigidBody *, std::vector<INTVL>> intervals_overlapping;

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
				}
			}
		}
		// add overlapping intervals to a list
		for (auto pair : intervals) {
			// note: pair.second is a ITVL type
			if (pair.second.overlap) {
				intervals_overlapping[pair.second.rb].push_back(pair.second);
				printf("overlap on interval:\n");
				printf("si: %f, ei:%f, dim:%d\n", pair.second.si, pair.second.ei, i);
			}
		}
	}

	// check if there there is a collision:
	for (auto pair : intervals_overlapping) {
		if (pair.second.size() == 2) {
			printf("overlap!\n");
		}
	}
}
