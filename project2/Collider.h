#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

enum BV
{
	obb = 1,
	plane = 2,
	sphere = 4
};

class Collider {
public:
	Collider();
	~Collider();

	virtual glm::vec3 getPos() { return m_pos; }
	BV getType() { return m_type; }

	virtual void setPos(glm::vec3 pos) { m_pos = pos; }

	virtual glm::mat3 getAxes() { return m_axes; }
	virtual glm::vec3 getRadii() { return m_radii; }

	void setAxes(glm::mat3 a) { m_axes = a; }
	void setAxes(int i, glm::vec3 a) { m_axes[i] = a; }
	void setRadii(glm::vec3 r) { m_radii = r; }
	void setRadii(int i, float r) { m_radii[i] = r; }

	void rotate(const float &angle, const glm::vec3 &vect);

	bool testCollision(Collider &b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen);

protected:
	void getClosestPtPointObb(glm::vec3 p, glm::vec3 &out);

	glm::vec3 closestPointsToObb(Collider &col, glm::vec3 verts[8], std::vector<int> &ins);

	bool findCollPointOBBOBB(Collider &b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen);
	bool findCollPointPlaneOBB(Collider &b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen);

	bool testOBBOBB(Collider &b);
	bool testOBBPlane(Collider &b);
	bool testOBBSpere(Collider &b);
	bool testPlanePlane(Collider &b);
	bool testPlaneSpere(Collider &b);
	bool testSpereSpere(Collider &b);

	glm::mat3 m_axes;
	glm::vec3 m_radii;
	glm::vec3 m_pos;
	BV m_type;
};
