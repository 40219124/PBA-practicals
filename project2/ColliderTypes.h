#pragma once
#include "Collider.h"

class Obb :
	public Collider
{
public:
	Obb();
	~Obb();
};

class Plane :
	public Collider
{
public:
	Plane();
	~Plane();
	glm::vec3 getPos();
	void setPos(glm::vec3 pos);
};

