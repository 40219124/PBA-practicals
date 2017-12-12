#include "ColliderTypes.h"
#include <glm/glm.hpp>

/*
*  OBB
*/

Obb::Obb()
{
	m_type = obb;
	m_pos = glm::vec3(0.0f);
	m_axes = glm::mat3(1.0f);
	m_radii = glm::vec3(1.0f);
}

Obb::~Obb() {}

/*
*  PLANE
*/

Plane::Plane()
{
	m_type = plane;
	m_pos[0] = 0;
	m_axes[0] = glm::vec3(0.0f, 1.0f, 0.0f);
}

Plane::~Plane() {}

glm::vec3 Plane::getPos() {
	return this->getPos()[0] * this->getAxes()[0];
}

void Plane::setPos(glm::vec3 pos) {
	this->setPos(glm::vec3(glm::dot(pos, this->getAxes()[0]), 0.0f, 0.0f));
}