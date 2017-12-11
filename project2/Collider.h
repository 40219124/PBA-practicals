#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

enum BV
{
	OBB = 1,
	Plane = 2,
	Sphere = 4
};

class Collider {
public:
	Collider();
	~Collider();

	glm::vec3 getPos() { return m_pos; }
	BV getType() { return m_type; }

	void setPos(glm::vec3 pos) { m_pos = pos; }

	glm::mat3 getAxes() { return m_axes; }
	glm::vec3 getRadii() { return m_radii; }

	void setAxes(glm::mat3 a) { m_axes = a; }
	void setAxes(int i, glm::vec3 a) { m_axes[i] = a; }
	void setRadii(glm::vec3 r) { m_radii = r; }
	void setRadii(int i, float r) { m_radii[i] = r; }

	void rotate(const float &angle, const glm::vec3 &vect);

	bool testCollision(Collider b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen);

protected:
	void getClosestPtPointObb(glm::vec3 p, glm::vec3 &out);

	bool findCollPointOBBOBB(Collider b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen);

	bool testOBBOBB(Collider b);
	bool testOBBPlane(Collider b);
	bool testOBBSpere(Collider b);
	bool testPlanePlane(Collider b);
	bool testPlaneSpere(Collider b);
	bool testSpereSpere(Collider b);

	glm::mat3 m_axes;
	glm::vec3 m_radii;
	glm::vec3 m_pos;
	BV m_type;
};
