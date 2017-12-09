#include "Obb.h"



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
