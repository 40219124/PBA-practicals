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
class Gravity : public Force {
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
class Drag : public Force {
public:
	Drag() {}
	// Physics
	glm::vec3 apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
private:
};

class Hooke : public Force {
public:
	Hooke() {}
	Hooke(Body *root, Body *end, float kstiff, float kdamp, float rest) {
		m_root = root;
		m_end = end;
		m_kstiff = kstiff;
		m_kdamp = kdamp;
		m_rest = rest;
	}

	Body* getRoot() { return m_root; }
	Body* getEnd() { return m_end; }
	
	float getRest() { return m_rest; }
	float getDamp() { return m_kdamp; }
	float getSpring() { return m_kstiff; }

	glm::vec3 apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);

private:
	float m_kstiff;
	float m_kdamp;
	float m_rest;

	Body *m_root;
	Body *m_end;
};