#include "RigidBody.h"

#include "./Eigen/Dense"

#include <memory>

/* might as well be a struct.. */ 
class Contact
{
public:
	std::shared_ptr<RigidBody> a;	// body with vertex
	std::shared_ptr<RigidBody> b;	// body with face
	Vector2d	p;					// vertex location
	Vector2d	n;					// normal to face
	//Vector2d	ea;	// edge direction for a
	//Vector2d	eb;	// edge direction for b
	//bool		vf;	// vertex face contact

	std::tuple<Vector2d, Vector2d> edge;
};
