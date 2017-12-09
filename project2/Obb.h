#pragma once
#include "Collider.h"

class Obb :
	public Collider
{
public:
	Obb();
	~Obb();

	glm::mat3 getAxes() { return m_axes; }
	glm::vec3 getRadii() { return m_radii; }

	void setAxes(glm::mat3 a) { m_axes = a; }
	void setAxes(int i, glm::vec3 a) { m_axes[i] = a; }
	void setRadii(glm::vec3 r) { m_radii = r; }
	void setRadii(int i, float r) { m_radii[i] = r; }

private:
	glm::mat3 m_axes;
	glm::vec3 m_radii;
};

