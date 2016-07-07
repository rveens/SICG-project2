#pragma once

#include "Particle.h"
#include "Force.h"

#include <memory>

class SpringForce : public Force {
public:
  SpringForce(std::shared_ptr<Particle> p1, std::shared_ptr<Particle> p2, double dist, double ks, double kd);
  virtual ~SpringForce();

  virtual void draw();
  virtual void calculateForce();

protected:

  std::shared_ptr<Particle> const m_p1;   // particle 1
  std::shared_ptr<Particle> const m_p2;   // particle 2 
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
