#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Mesh.h"
#include "Force.h"
#include "ColliderTypes.h"

class Body
{
public:
	Body();
	~Body();

	/*
	** GET METHODS
	*/
	// mesh
	Mesh& getMesh() { return m_mesh; }
	Collider& getColl() { return m_coll; }

	// transform matrices
	glm::mat4 getTranslate() const { return m_mesh.getTranslate(); }
	glm::mat4 getRotate() const { return m_mesh.getRotate(); }
	glm::mat4 getScale() const { return m_mesh.getScale(); }

	// dynamic variables
	glm::vec3& getAcc() { return m_acc; }
	glm::vec3& getVel() { return m_vel; }
	glm::vec3& getPos() { return m_pos; }


	// physical properties
	float getMass() const { return m_mass; }
	float getCor() { return m_cor; }

	bool getFixed() { return m_fixed; }
	bool getCollFixed() { return m_collFixed; }

	glm::vec3 applyForces(glm::vec3 pos, glm::vec3 vel, float totalTime, float deltaTime);

	/*
	** SET METHODS
	*/
	// mesh
	void setMesh(Mesh m) { m_mesh = m; }
	void setColl(Collider c) { m_coll = c; }

	// dynamic variables
	void setAcc(const glm::vec3 &vect) { m_acc = vect; }
	void setVel(const glm::vec3 &vect) { m_vel = vect; }
	void setVel(int i, float v) { m_vel[i] = v; } //set the ith coordinate of the velocity vector
	void setPos(const glm::vec3 &vect) { m_pos = vect; m_mesh.setPos(vect); }
	void setPos(int i, float p) { m_pos[i] = p; m_mesh.setPos(i, p); } //set the ith coordinate of the position vector
	void setRotate(const glm::mat4 &rot) { m_mesh.setRotate(rot); m_coll.setAxes(glm::mat3(rot)); }

	// physical properties
	void setCor(float cor) { m_cor = cor; }
	void setMass(float mass) { m_mass = mass; }
	void setFixed(bool b) { m_fixed = b; }
	void setCollFixed(bool b) { m_collFixed = b; }

	// queues
	void addTlate(glm::vec3 t) { m_tlateQueue.push_back(t); }
	void addVel(glm::vec3 v) { m_velQueue.push_back(v); }
	virtual void resolveQueues();

	/*
	** OTHER METHODS
	*/

	// transformation methods
	void translate(const glm::vec3 &vect);
	void rotate(float angle, const glm::vec3 &vect);
	void scale(const glm::vec3 &vect);

	std::vector<Force*> getForces() { return m_forces; }

	void addForce(Force *force) { m_forces.push_back(force); }

private:
	Mesh m_mesh; // mesh used to represent the body
	Collider m_coll; // collider used to break everything

	std::vector<glm::vec3> m_tlateQueue;
	std::vector<glm::vec3> m_velQueue;

	float m_mass; // mass
	float m_cor; // coefficient of restitution

	glm::vec3 m_acc; // acceleration
	glm::vec3 m_vel; // velocity
	glm::vec3 m_pos; // position

	bool m_fixed = false;
	bool m_collFixed = false;

	std::vector<Force*> m_forces;
};

