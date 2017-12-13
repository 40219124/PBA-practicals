#pragma once
// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>
#include <vector>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include "glm/ext.hpp"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Mesh.h"
#include "Particle.h"
#include "RigidBody.h"
#include "ColliderTypes.h"

// time
float t = 0.0f;
const float dt = 0.01f;
float timeMultiplier = 0.86f; // controls the speed of the simulation

// forces
Gravity g = Gravity(glm::vec3(0.0f, -9.8f, 0.0f));

// integration: returns the position difference from the acceleration and a timestep
void integratePos(RigidBody &rb, float t, float dt) {
	rb.setVel(rb.getVel() + dt * rb.getAcc());
	glm::vec3 dPos = dt * rb.getVel();
	rb.translate(dPos);
}

// integration: returns the rotation difference from the acceleration and a timestep
void integrateRot(RigidBody &rb, float dt) {
	rb.setAngVel(rb.getAngVel() + dt * rb.getAngAcc());
	glm::mat3 angVelSkew = glm::matrixCross3(rb.getAngVel());
	// create 3x3 rotation matrix from rb rotation matrix
	glm::mat3 R = glm::mat3(rb.getRotate());
	// update rotation matrix
	R += dt*angVelSkew*R;
	R = glm::orthonormalize(R);
	rb.setRotate(glm::mat4(R));
}

void ApplyImpulse(RigidBody &rb, const glm::vec3 &impLoc, const glm::vec3 &impDir) {
	glm::vec3 deltaV = impDir / rb.getMass();
	glm::vec3 deltaW = rb.getInvInertia() * (glm::cross(impLoc, impDir));
	rb.addVel(deltaV);
	rb.addAngVel(deltaW);
}

void ApplyFriction(RigidBody &rb, const glm::vec3 &collP, const glm::vec3 &norm, const glm::vec3 &vr, const float jn) {
	float mu = 0.25f;
	glm::vec3 vt = vr - glm::dot(vr, norm) * norm;
	if (glm::dot(vt, vt) > 0.0f) {
		glm::vec3 jt = -mu * jn * vt / glm::length(vt);
		float lenJt = glm::dot(jt, jt);
		if (lenJt > 16.0f) {
			jt /= sqrtf(lenJt);
			jt *= 4.0f;
		}
		if (glm::dot(jt, jt) > glm::dot(vt, vt)) {
			jt /= glm::length(jt);
			jt *= glm::length(vt);
		}
		/*float maxFW = glm::length(rb.getAngVel() / glm::length(rb.getInvInertia() * glm::cross(collP, vt / glm::length(vt))));
		if (glm::length(jt) > maxFW) {
			jt = jt / glm::length(jt);
			jt *= maxFW;
		}*/
		ApplyImpulse(rb, collP, jt);
	}
}

void ApplyCollisionFixed(RigidBody &fix, RigidBody &mov, Particle &p1) {
	glm::vec3 collP = glm::vec3(0.0f);
	glm::vec3 collN = glm::vec3(0.0f);
	float halfPen = 0.0f;
	bool flag = fix.getColl().testCollision(mov.getColl(), collP, collN, halfPen);
	if (flag) {
		mov.setCollFixed(true);
		// Revert some movement
		glm::vec3 trans = halfPen * 2.0f * collN;
		mov.addTlate(trans);
		glm::vec3 movPos = mov.getPos() + trans;
		collP += halfPen * 2.0f * collN;
		p1.setPos(collP);
		// Set up loop variables
		glm::vec3 cToP = collP - movPos;
		glm::vec3 vAtP = mov.getVel() + glm::cross(mov.getAngVel(), cToP);
		float oneOverM = 1.0f / mov.getMass();
		glm::mat3 inT = mov.getInvInertia();
		float deBit = glm::dot(collN, glm::cross(inT * glm::cross(cToP, collN), cToP));
		// Compute j
		float numer = -(1.0f + mov.getCor()) * glm::dot(vAtP, collN);
		float denom = oneOverM + deBit;
		float j = numer / denom;
		// Set new velocities
		mov.addVel((j * oneOverM) * collN);
		mov.addAngVel(j * inT * glm::cross(cToP, collN));
		ApplyFriction(mov, collP - movPos, collN, vAtP, j);
	}
}

void ApplyCollision(RigidBody &rb1, RigidBody &rb2, Particle &p1) {
	glm::vec3 collP = glm::vec3(0.0f);
	glm::vec3 collN = glm::vec3(0.0f);
	float halfPen = 0.0f;
	bool flag = rb1.getColl().testCollision(rb2.getColl(), collP, collN, halfPen);
	if (flag) {
		if (rb1.getFixed() || rb2.getFixed()) {
			ApplyCollisionFixed((rb1.getFixed() ? rb1 : rb2), (rb1.getFixed() ? rb2 : rb1), p1);
		}
		else {
			p1.setPos(collP);
			// Collision response time
			RigidBody* rbs[2];
			// Allocate rigid bodies based on face collision (rbs[0] is the face)
			if (glm::dot(collP - rb1.getPos(), collN) > 0) {
				rbs[0] = &rb1;
				rbs[1] = &rb2;
			}
			else {
				rbs[1] = &rb1;
				rbs[0] = &rb2;
			}
			glm::vec3 cPos[2];
			// Revert some movement
			for (int i = 0; i < 2; i++) {
				glm::vec3 trans = (i == 0 ? -halfPen : halfPen) * collN;
				if (rbs[i]->getCollFixed()) {
					trans *= 0.0f;
				}
				if (rbs[(i + 1) % 2]->getCollFixed()) {
					trans *= 2.0f;
				}
				rbs[i]->addTlate(trans);
				cPos[i] = rbs[i]->getPos() + trans;
			}
			// Set up loop variables
			glm::vec3 cToP[2];
			glm::vec3 vAtP[2];
			float oneOverM[2];
			glm::mat3 inT[2];
			float deBit[2];
			// Get variables from different rb's
			for (int i = 0; i < 2; i++) {
				cToP[i] = collP - cPos[i];
				vAtP[i] = rbs[i]->getVel() + glm::cross(rbs[i]->getAngVel(), cToP[i]);
				oneOverM[i] = 1.0f / rbs[i]->getMass();
				inT[i] = rbs[i]->getInvInertia();
				deBit[i] = glm::dot(collN, glm::cross(inT[i] * glm::cross(cToP[i], collN), cToP[i]));
			}
			// Compute j
			float numer = -(1.0f + rbs[0]->getCor()) * glm::dot(vAtP[1] - vAtP[0], collN);
			float denom = oneOverM[0] + oneOverM[1] + deBit[0] + deBit[1];
			float j = numer / denom;
			// Apply new forces
			for (int i = 0; i < 2; i++) {
				// Set new velocities
				rbs[i]->addVel(-(j * oneOverM[i]) * collN);
				glm::vec3 newAV = j * inT[i] * glm::cross(cToP[i], collN);
				rbs[i]->addAngVel(-newAV);
				j *= -1;
			}
			ApplyFriction(rb1, collP - cPos[0], collN, vAtP[1] - vAtP[0], -j);
			ApplyFriction(rb2, collP - cPos[1], collN, vAtP[1] - vAtP[0], j);
		}
	}
}

// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 3.0f, 20.0f));

	// create environment ( large plane at y=-3)
	float planeScale = 40.0f;
	RigidBody rbPlane = RigidBody();
	Mesh plane = Mesh::Mesh(Mesh::QUAD);
	rbPlane.setColl(Plane::Plane());
	plane.setShader(Shader("resources/shaders/physics.vert", "resources/shaders/transp.frag"));
	plane.scale(glm::vec3(planeScale));
	plane.translate(glm::vec3(0.0f, 0.0f, 0.0f));
	rbPlane.setMesh(plane);
	rbPlane.setMass(10.0f);
	rbPlane.setFixed(true);

	std::vector<RigidBody> planes;

	for (int i = 0; i < 4; ++i) {
		RigidBody sidePlane = RigidBody();
		Mesh mBackPlane = Mesh::Mesh(Mesh::QUAD);
		sidePlane.setColl(Plane::Plane());
		glm::vec3 pnorm = glm::vec3((i < 2 ? 1.0f : 0.0f), 0.0f, (i >= 2 ? 1.0f : 0.0f));
		if (i % 2 == 0) {
			pnorm *= -1;
		}
		sidePlane.getColl().setAxes(0, pnorm);
		sidePlane.setMesh(mBackPlane);
		sidePlane.translate(-pnorm * planeScale);
		sidePlane.setMass(10.0f);
		sidePlane.setFixed(true);
		planes.push_back(sidePlane);
	}
	Shader rbShader = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	
	// Create all dominos
	std::vector<RigidBody> dominos;
	int domI = 30;
	float rad = 10.0f;
	for (int i = 0; i < domI; ++i) {
		Mesh meh = Mesh::Mesh(Mesh::CUBE);
		RigidBody rib = RigidBody();
		rib.setMesh(meh);
		rib.setColl(Obb::Obb());
		rib.getMesh().setShader(rbShader);
		rib.setMass(1.0f);
		float gap = 1.4f;
		rib.translate(glm::vec3(gap * (-domI / 2.0f) + i * gap, 1.1f, 0.0f));
		//rib.translate(glm::vec3(rad * cos(M_PI * i * 2.0f / domI), 2.0f * rad + rad * sin(M_PI * i * 2.0f / domI), -10.0f + i * 0.1f));
		rib.setVel(glm::vec3(0.0f));
		//rib.setAngVel(glm::vec3(0.0f, 0.0f, 0.1f));
		rib.setAngVel(glm::vec3(0.0f, 0.0f, 0.0f));
		rib.setCor(0.6f);
		rib.scale(glm::vec3(0.2f, 1.0f, 0.4f));
		rib.addForce(new Gravity(glm::vec3(0.0f, -9.8f * rib.getMass(), 0.0f)));
		dominos.push_back(rib);
	}
	ApplyImpulse(dominos[0], glm::vec3(-0.2f, 1.0f, 0.0f), glm::vec3(5.0f, 0.0f, 0.0f));


	// Create particles
	Shader pShader = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");
	Particle p1 = Particle::Particle();
	p1.setMesh(Mesh::Mesh(Mesh::MeshType::TRIANGLE));
	p1.translate(glm::vec3(-1.0f, 3.0f, -3.0f));
	p1.getMesh().setShader(pShader);
	p1.scale(glm::vec3(0.25f));

	// time
	float currentTime = (float)glfwGetTime();
	float timeAccumulator = 0.0f;

	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// physics time
		float newTime = (float)glfwGetTime();
		float frameTime = newTime - currentTime;
		currentTime = newTime;
		timeAccumulator += (frameTime * timeMultiplier);
		// Manage interaction
		app.doMovement(frameTime);

		while (timeAccumulator >= dt) {

			/*
			**	SIMULATION
			*/
			if (!Application::pauseSimulation) {

				// Accelerations before new velocities
				for (int i = 0; i < dominos.size(); ++i) {
					dominos[i].setAcc(dominos[i].applyForces(dominos[i].getPos(), dominos[i].getVel(), t, dt));
				}
				// Integration (position)
				for (int i = 0; i < dominos.size(); ++i) {
					integratePos(dominos[i], t, dt);
				}

				// Integration (rotation)
				for (int i = 0; i < dominos.size(); ++i) {
					integrateRot(dominos[i], dt);
				}

				// Collisions
				for (int i = 0; i < dominos.size() - 1; ++i) {
					// Planes first so you know if domino can't move
					ApplyCollision(dominos[i], rbPlane, p1);
					// collision plane loop
					for (int j = 0; j < planes.size(); ++j) {
						ApplyCollision(dominos[i], planes[j], p1);
					}
					// Domino to domino loop (checks ahead only)
					for (int j = i + 1; j < dominos.size(); ++j) {
						ApplyCollision(dominos[i], dominos[j], p1);
					}
				}

				// Resolve queues
				for (int i = 0; i < dominos.size(); ++i) {
					dominos[i].resolveQueues();
				}
			}
			timeAccumulator -= dt;
			t += dt;
		}

		/*
		**	RENDER
		*/
		// clear buffer
		app.clear();

		// draw groud plane
		app.draw(plane);

		// draw rigid body
		app.draw(p1.getMesh());
		for (int i = 0; i < dominos.size() - 1; ++i) {
			app.draw(dominos[i].getMesh());
		}

		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

