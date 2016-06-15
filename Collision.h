#include "RigidBody.h"

#include "./Eigen/Dense"

/* might as well be a struct.. */ 
class Collision
{
public:
	RigidBody	*a;	// body with vertex
	RigidBody	*b;	// body with face
	Vector2d	p;	// vertex location
	Vector2d	n;	// normal to face
	Vector2d	ea;	// edge direction for a
	Vector2d	eb;	// edge direction for b
	bool		vf;	// vertex face contact
};
