#include "RigidBody.h"

RigidBody::RigidBody() {}

RigidBody::~RigidBody() {}

void RigidBody::setMass(float mass) {
	Body::setMass(mass);
	m_invInertia = calcIT();
}

void RigidBody::setAngVel(const glm::vec3 &omega) {
	glm::vec3 o = omega;
	float lim = 30.0f;
	if (glm::dot(o, o) > powf(lim, 2.0f)) {
		o /= glm::length(o);
		o *= lim;
	}
	m_angVel = o;
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

void RigidBody::resolveQueues() {
	Body::resolveQueues();
	this->m_hit.clear();
	glm::vec3 a = glm::vec3(0.0f);
	while (!m_angQueue.empty()) {
		a += m_angQueue.back();
		m_angQueue.pop_back();
	}
	this->setAngVel(this->getAngVel() + a);
}


bool RigidBody::hasHit(RigidBody* r) {
	if (!m_hit.empty()) {
		for (int i = 0; i < m_hit.size(); ++i) {
			if (m_hit[i] == r) {
				return true;
			}
		}
	}
	return false;
}
