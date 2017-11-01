#include "RigidBody.h"

RigidBody::RigidBody() {}

RigidBody::~RigidBody() {}

void RigidBody::scale(glm::vec3 scale) {
	for (int i = 0; i < 3; ++i) {
		m_dimensions[i] *= scale[i];
	}
	m_invInertia = glm::inverse(calcIT());
	Body::scale(scale);
}

glm::mat3 RigidBody::calcIT() {
	glm::mat3 it = glm::mat3(0.0f);
	it[0][0] = getMass() * (powf(m_dimensions[1], 2.0f) + powf(m_dimensions[2], 2.0f)) / 12.0f;
	it[1][1] = getMass() * (powf(m_dimensions[0], 2.0f) + powf(m_dimensions[2], 2.0f)) / 12.0f;
	it[2][2] = getMass() * (powf(m_dimensions[0], 2.0f) + powf(m_dimensions[1], 2.0f)) / 12.0f;
}
