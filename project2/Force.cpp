#include <iostream>
#include <cmath>
#include "Force.h"
#include "Body.h"
#include "glm/ext.hpp"

glm::vec3 Force::apply(const float totalTime, const glm::vec3 &pos, const glm::vec3 &vel) {
	return glm::vec3(0.0f);
}
/*
Gravity
*/
glm::vec3 Gravity::apply(const float totalTime, const glm::vec3 &pos, const glm::vec3 &vel) {
	return m_gravity;
}
/*
Drag
*/
glm::vec3 Drag::apply(const float totalTime, const glm::vec3 &pos, const glm::vec3 &vel) {
	return (-glm::length2(vel) / 900.0f) * vel;
}
/*
Hooke
*/
glm::vec3 Hooke::apply(const float totalTime, const glm::vec3 &pos, const glm::vec3 &vel) {
	glm::vec3 result = glm::vec3(0.0f);
	if (totalTime != this->m_tTime) {
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

		result = ((-this->getStiff() * (this->getRest() - glm::length(gap))) - (this->getDamp() * (velRoot - velEnd))) * gapUnit;

		if (getEnd()->getPos() == pos) {
			result = -result;
		}

		this->m_result = result;
		this->m_tTime = totalTime;
	}
	else {
		result = -this->m_result;
	}
	return result;
}
/*
Wind
*/
glm::vec3 Wind::apply(const float totalTime, const glm::vec3 &pos, const glm::vec3 &vel) {
	glm::vec3 result = glm::vec3(0.0f);
	if (totalTime != m_tTime) {
		glm::vec3 s1 = m_p2->getPos() - m_p1->getPos();
		glm::vec3 s2 = m_p3->getPos() - m_p1->getPos();
		glm::vec3 norm = s1 * s2;
		if (norm != glm::vec3(0.0f)) {
			glm::vec3 triVel = (m_p1->getVel() + m_p2->getVel() + m_p3->getVel()) / 3.0f;
			glm::vec3 dV = triVel - *m_airSpeed;
			if (dV != glm::vec3(0.0f)) {
				float area = length(norm) / 2.0f;
				norm = norm / glm::length(norm);
				float aEff = area * (glm::dot(dV, norm)) / length(dV);
				result = length(dV) * length(dV) * aEff * (-norm);
				result = result / 3.0f;
			}
		}
		m_result = result;
		m_tTime = totalTime;
	}
	else {
		result = m_result;
	}
	return result;
}