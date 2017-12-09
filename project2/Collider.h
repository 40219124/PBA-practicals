#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

enum BV
{
	OBB,
	Plane,
	Sphere
};

class Collider {
public:
	Collider();
	~Collider();

	glm::vec3 getPos() { return m_pos; }
	BV getType() { return m_type; }

	void setPos(glm::vec3 pos) { m_pos = pos; }

protected:
	glm::vec3 m_pos;
	BV m_type;
};
