#pragma once

#include "RigidBody.h"
#include "Force.h"
#include <memory>

class MouseForce : public Force {
public:
	MouseForce(std::shared_ptr<RigidBody> rb, Vector2d &mousePos, double strength, bool mouse_dragged);
	virtual ~MouseForce();

	virtual void draw();
	virtual void calculateForce();

	std::shared_ptr<RigidBody> m_rb;			// Rigid Body force acts on
	Vector2d m_mousePos;		// Mouse position
	double const m_strength;    // strength multiplier of force
	bool m_mouse_dragged;		// indicates whether mouse is currently being dragged (i.e. force must be calculated)
};
