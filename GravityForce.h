#include "Force.h"
#include "RigidBody.h"


class GravityForce : public Force
{
public:
	GravityForce(RigidBody *rb);
	virtual ~GravityForce();

	void calculateForce();
	void draw();
private:
	const double m_g = 9.81;
	RigidBody *const m_rb;
};
