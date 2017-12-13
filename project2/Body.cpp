#include "Body.h"

Body::Body()
{
}


Body::~Body()
{
}

/* TRANSFORMATION METHODS*/
void Body::translate(const glm::vec3 &vect) {
	m_pos = m_pos + vect;
	m_mesh.translate(vect);
	m_coll.setPos(m_pos);
}

void Body::rotate(float angle, const glm::vec3 &vect) {
	m_mesh.rotate(angle, vect);
	m_coll.rotate(angle, vect);
}

void Body::scale(const glm::vec3 &vect) {
	m_mesh.scale(vect);
	for (int i = 0; i < 3; ++i) {
		m_coll.setRadii(i, m_coll.getRadii()[i] * vect[i]);
	}
}

glm::vec3 Body::applyForces(glm::vec3 pos, glm::vec3 vel, float totalTime, float deltaTime) {
	glm::vec3 fAccumulator = glm::vec3(0.0f);

	for (auto &f : m_forces) {
		fAccumulator += f->apply(totalTime, pos, vel);
	}
	glm::vec3 acc = fAccumulator / getMass();
	if (glm::length(acc) > 1000.0f) {
		acc = 1000.0f * acc / length(acc);
	}
	return acc;
}

void Body::resolveQueues() {
	glm::vec3 t = glm::vec3(0.0f);
	while (!m_tlateQueue.empty()) {
		t += m_tlateQueue.back();
		m_tlateQueue.pop_back();
	}
	this->translate(t);
	glm::vec3 v = glm::vec3(0.0f);
	while (!m_velQueue.empty()) {
		v += m_velQueue.back();
		m_velQueue.pop_back();
	}
	this->setVel(this->getVel() + v);
}