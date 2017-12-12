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
float timeMultiplier = 1.6f; // controls the speed of the simulation

// forces
Gravity g = Gravity(glm::vec3(0.0f, -9.8f, 0.0f));

// integration: returns the position difference from the acceleration and a timestep
void integratePos(RigidBody &rb, float t, float dt) {
	rb.setAcc(rb.applyForces(rb.getPos(), rb.getVel(), t, dt));
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
	rb.setVel(rb.getVel() + deltaV);
	rb.setAngVel(rb.getAngVel() + deltaW);
}

void ApplyCollision(RigidBody &rb1, RigidBody &rb2, Particle &p1) {
	glm::vec3 collP = glm::vec3(0.0f);
	glm::vec3 collN = glm::vec3(0.0f);
	float halfPen = 0.0f;
	bool flag = rb1.getColl().testCollision(rb2.getColl(), collP, collN, halfPen);
	if (flag) {
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
		// Set up loop variables
		glm::vec3 cToP[2];
		glm::vec3 vAtP[2];
		float oneOverM[2];
		glm::mat3 inT[2];
		float deBit[2];
		// Get variables from different rb's
		for (int i = 0; i < 2; i++) {
			cToP[i] = collP - rbs[i]->getPos();
			vAtP[i] = rbs[i]->getVel() + glm::cross(rbs[i]->getAngVel(), cToP[i]);
			oneOverM[i] = 1.0f / rbs[i]->getMass();
			inT[i] = rbs[i]->getInvInertia();
			deBit[i] = glm::dot(collN, glm::cross(inT[i] * glm::cross(cToP[i], collN), cToP[i]));
		}
		// Compute j
		float numer = -(1.0f + 0.6f) * glm::dot(vAtP[1] - vAtP[0], collN);
		float denom = oneOverM[0] + oneOverM[1] + deBit[0] + deBit[1];
		float j = numer / denom;
		// Apply new forces
		for (int i = 0; i < 2; i++) {
			// Revert some movement
			glm::vec3 trans = (i == 0 ? -halfPen : halfPen) * collN;
			rbs[i]->translate(trans);
			// Set new velocities
			rbs[i]->setVel(rbs[i]->getVel() - (j * oneOverM[i]) * collN);
			rbs[i]->setAngVel(rbs[i]->getAngVel() - j*inT[i] * glm::cross(cToP[i], collN));
			j *= -1;
		}


		//app.pauseSimulation = true;
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
	RigidBody rbPlane = RigidBody();
	Mesh plane = Mesh::Mesh(Mesh::QUAD);
	plane.setShader(Shader("resources/shaders/physics.vert", "resources/shaders/transp.frag"));
	plane.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	plane.translate(glm::vec3(0.0f, 0.0f, 0.0f));
	rbPlane.setMesh(plane);
	rbPlane.setColl(Plane::Plane());

	// create cube
	Mesh m1 = Mesh::Mesh(Mesh::CUBE);
	// rigid body set up
	RigidBody rb1 = RigidBody();
	rb1.setMesh(m1);
	Shader rbShader = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	rb1.getMesh().setShader(rbShader);
	rb1.setMass(1.0f);
	rb1.translate(glm::vec3(5.0f, 3.0f, 0.2f));
	//rb1.setVel(glm::vec3(-2.0f, 0.0f, 0.0f));
	rb1.setAngVel(glm::vec3(0.5f, 0.5f, 0.0f));
	rb1.setColl(Obb::Obb());

	// create rb2
	Mesh m2 = Mesh::Mesh(Mesh::CUBE);
	RigidBody rb2 = RigidBody();
	rb2.setMesh(m2);
	rb2.getMesh().setShader(rbShader);
	rb2.setMass(1.0f);
	rb2.translate(glm::vec3(-5.0f, 3.2f, 0.0f));
	rb2.setVel(glm::vec3(1.0f, 0.0f, 0.0f));
	rb2.setAngVel(glm::vec3(0.1f, -0.6f, 0.0f));
	rb2.setColl(Obb::Obb());

	// create rb3
	Mesh m3 = Mesh::Mesh(Mesh::CUBE);
	RigidBody rb3 = RigidBody();
	rb3.setMesh(m3);
	rb3.getMesh().setShader(rbShader);
	rb3.setMass(1.0f);
	rb3.translate(glm::vec3(-15.0f, 3.3f, 0.0f));
	rb3.setVel(glm::vec3(2.0f, 0.0f, 0.0f));
	rb3.setAngVel(glm::vec3(0.2f, -0.6f, 0.4f));
	rb3.setColl(Obb::Obb());

	if (true) {

	}

	Shader pShader = Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag");
	Particle p1 = Particle::Particle();
	p1.setMesh(Mesh::Mesh(Mesh::MeshType::TRIANGLE));
	p1.translate(glm::vec3(-1.0f, 3.0f, -3.0f));
	p1.getMesh().setShader(pShader);
	p1.scale(glm::vec3(0.25f));
	Particle p2 = Particle::Particle();
	p2.setMesh(Mesh::Mesh(Mesh::MeshType::TRIANGLE));
	p2.translate(glm::vec3(1.0f, 3.0f, -3.0f));
	p2.getMesh().setShader(pShader);
	p2.scale(glm::vec3(0.25f));
	Particle p3 = Particle::Particle();
	p3.setMesh(Mesh::Mesh(Mesh::MeshType::TRIANGLE));
	p3.translate(glm::vec3(1.0f, 3.0f, -3.0f));
	p3.getMesh().setShader(pShader);
	p3.scale(glm::vec3(0.25f));

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


				//Integration (position)
				integratePos(rb1, t, dt);
				integratePos(rb2, t, dt);
				integratePos(rb3, t, dt);

				// integration (rotation)
				integrateRot(rb1, dt);
				integrateRot(rb2, dt);
				integrateRot(rb3, dt);

				
				ApplyCollision(rb1, rb2, p1);
				ApplyCollision(rb1, rb3, p2);
				ApplyCollision(rb2, rb3, p3);



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
		app.draw(rb1.getMesh());
		app.draw(rb2.getMesh());
		app.draw(rb3.getMesh());
		app.draw(p1.getMesh());
		app.draw(p2.getMesh());
		app.draw(p3.getMesh());

		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

