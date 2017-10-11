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
	if (gapLen == 0) {
		return glm::vec3(0.0f);
	}
	glm::vec3 gapUnit = gap / (glm::length(gap));
	float velRoot = glm::dot(gapUnit, this->getRoot()->getVel());
	float velEnd = glm::dot(gapUnit, this->getEnd()->getVel());

	return ((-this->getStiff() * (this->getRest() - glm::length(gap))) - (this->getDamp() * (velRoot - velEnd))) * gapUnit;
}