#include "Integrator.h"

class EulerStep : public Integrator
{
public:
	EulerStep();
	virtual ~EulerStep();

	void integrate(std::shared_ptr<SolverInterface> rb, double timeStep);
	std::string getString();
};
