#include <iostream>
#include <cmath>
#include "Force.h"
#include "Body.h"
#include "glm/ext.hpp"

glm::vec3 Force::apply(const glm::vec3 &pos, const glm::vec3 &vel) {
	return glm::vec3(0.0f);
}
/*
Gravity
*/
glm::vec3 Gravity::apply(const glm::vec3 &pos, const glm::vec3 &vel) {
	return m_gravity;
}
/*
Drag
*/
glm::vec3 Drag::apply(const glm::vec3 &pos, const glm::vec3 &vel) {
	return (-glm::length2(vel) / 900.0f) * vel;
}
/*
Hooke
*/
glm::vec3 Hooke::apply(const glm::vec3 &pos, const glm::vec3 &vel) {
	glm::vec3 gap = this->getEnd()->getPos() - this->getRoot()->getPos();
	float gapLen = glm::length(gap);
	glm::vec3 gapUnit;
	if (gapLen == 0) {
		gapUnit = glm::vec3(0.0f);
	}
	else {
		gapUnit = gap / (glm::length(gap));
	}
	float velRoot = glm::dot(gapUnit, this->getRoot()->getVel());
	float velEnd = glm::dot(gapUnit, this->getEnd()->getVel());

	return ((-this->getStiff() * (this->getRest() - glm::length(gap))) - (this->getDamp() * (velRoot - velEnd))) * gapUnit;
}
/*
Wind
*/
glm::vec3 Wind::apply(const glm::vec3 &pos, const glm::vec3 &vel) {
	glm::vec3 result = glm::vec3(0.0f);
	if (m_cycle == 0) {
		glm::vec3 s1 = m_p2->getPos() - m_p1->getPos();
		glm::vec3 s2 = m_p3->getPos() - m_p1->getPos();
		glm::vec3 norm = s1 * s2;
		if (norm != glm::vec3(0.0f)) {
			glm::vec3 triVel = (m_p1->getVel() + m_p2->getVel() + m_p3->getVel) / 3.0f;
			glm::vec3 dV = triVel - *m_airSpeed;
			if (dV != glm::vec3(0.0f)) {
				norm = norm / glm::length(norm);
				float area = (length(s1) * length(s2)) / 2.0f;
				float aEff = area * (glm::dot(dV, norm)) / length(dV);
				result = length(dV) * length(dV) * aEff * (-norm);
				result = result / 3.0f;
			}
		}
		m_result = result;
	}
	else {
		result = m_result;
	}
	Cycle();
	return result;
}