#pragma once
#include<glm/glm.hpp>
#include<iostream>


class Body; // Forward declaration to avoid circular dependencies

class Force
{
public:
	Force() {}
	~Force() {}
	virtual glm::vec3 apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
};
/*
Gravity Class
*/
class Gravity :public Force {
public:
	// Constructors
	Gravity() {}
	Gravity(const glm::vec3 &gravity) { m_gravity = gravity; }
	// Get and set methods
	glm::vec3 getGravity() const { return m_gravity; }
	void setGravity(glm::vec3 gravity) { m_gravity = gravity; }
	// Physics
	glm::vec3 apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
private:
	glm::vec3 m_gravity = glm::vec3(0.0f, -9.8f, 0.0f);
};
/*
Drag Class
*/
class Drag :public Force {
public:
	Drag(){}
	// Physics
	glm::vec3 apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
private:

};