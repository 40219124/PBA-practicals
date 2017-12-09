#include "Obb.h"
#include <glm/glm.hpp>



Obb::Obb()
{
	m_type = OBB;
	m_pos = glm::vec3(0.0f);
	m_axes = glm::mat3(1.0f);
	m_radii = glm::vec3(1.0f);
}


Obb::~Obb()
{
}

void Obb::rotate(const float &angle, const glm::vec3 &vect) {
	m_axes = glm::rotate(glm::mat4(m_axes), angle, vect);
}