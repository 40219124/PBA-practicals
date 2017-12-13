#pragma once
#include "Body.h"

class RigidBody :
	public Body
{
public:
	RigidBody();
	~RigidBody();
	// set and get methods
	void setAngVel(const glm::vec3 &omega) { m_angVel = omega; }
	void setAngAccl(const glm::vec3 &alpha) { m_angAcc = alpha; }
	void setInvInertia(const glm::mat3 &invInertia) { m_invInertia = invInertia; }
	void setMass(float mass);

	glm::vec3 getAngVel() { return m_angVel; }
	glm::vec3 getAngAcc() { return m_angAcc; }
	glm::mat3 getInvInertia() { return glm::mat3(getRotate()) * m_invInertia * glm::mat3(glm::transpose(getRotate())); }
	void scale(glm::vec3 vect);

	void addAngVel(glm::vec3 a) { m_angQueue.push_back(a); }
	void resolveQueues();
	
private:
	glm::mat3 calcIT();

	std::vector<glm::vec3> m_angQueue;

	float m_density;
	glm::vec3 m_dimensions = glm::vec3(2.0f, 2.0f, 2.0f);
	glm::mat3 m_invInertia; // Inverse Inertia
	glm::vec3 m_angVel; // angular velocity
	glm::vec3 m_angAcc; // angular acceleration
};