#include "RigidBody.h"

RigidBody::RigidBody() {}

RigidBody::~RigidBody() {}

void RigidBody::setMass(float mass) {
	Body::setMass(mass);
	m_invInertia = calcIT();
}

void RigidBody::scale(glm::vec3 scale) {
	Body::scale(scale);
	for (int i = 0; i < 3; ++i) {
		m_dimensions[i] *= scale[i];
	}
	m_invInertia = calcIT();
}

glm::mat3 RigidBody::calcIT() {
	glm::mat3 it = glm::mat3(0.0f);
	float mass12 = getMass() / 12.0f;
	it[0][0] = (powf(m_dimensions[1], 2.0f) + powf(m_dimensions[2], 2.0f)) * mass12;
	it[1][1] = (powf(m_dimensions[0], 2.0f) + powf(m_dimensions[2], 2.0f)) * mass12;
	it[2][2] = (powf(m_dimensions[0], 2.0f) + powf(m_dimensions[1], 2.0f)) * mass12;
	return glm::inverse(it);
}
