#include "Integrator.h"

class EulerStep : public Integrator
{
public:
	EulerStep();
	virtual ~EulerStep();

	void integrate(SolverInterface *rb, double timeStep);
	std::string getString();
};
