#pragma once

class Force
{
public:
	virtual void calculateForce() = 0;
	virtual void draw() = 0;

	virtual ~Force() { }
};
