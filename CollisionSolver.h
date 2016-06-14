#include "Collision.h"

#include <vector>

class CollisionSolver
{
public:
	CollisionSolver();
	virtual ~CollisionSolver();

	void detectCollisions(std::vector<RigidBody *> &rbodies);
private:
	std::vector<Collision> m_Collisions;
};
