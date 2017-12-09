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
#include "Obb.h"


// time
float t = 0.0f;
const float dt = 0.01f;
float timeMultiplier = 2.0f; // controls the speed of the simulation

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

bool checkCollision(Obb &obb1, Obb &obb2) {
	float r1, r2;
	glm::mat3 rot, absRot;
	float error = 0.00001;

	// Projection of 2's axes onto 1's axes
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rot[i][j] = glm::dot(obb1.getAxes()[i], obb2.getAxes()[j]);
		}
	}
	// Difference between two centres
	glm::vec3 t = obb2.getPos() - obb1.getPos();
	// t in 1's space (coordinate frame)
	t = glm::vec3(glm::dot(t, obb1.getAxes()[0]), glm::dot(t, obb1.getAxes()[1]), glm::dot(t, obb1.getAxes()[2]));

	// Compute common subexpressions
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			absRot[i][j] = abs(rot[i][j]) + error;
		}
	}

	// Test axes L = 1[0], L = 1[1], L = 1[2]
	for (int i = 0; i < 3; ++i) {
		r1 = obb1.getRadii[i];
		r2 = obb2.getRadii[0] * absRot[i][0] + obb2.getRadii[1] * absRot[i][1] + obb2.getRadii[2] * absRot[i][2];
		if (abs(t[i]) > (r1 + r2)) { return false; }
	}
	// Test axes L = 2[0], L = 2[1], L = 2[2]
	for (int i = 0; i < 3; ++i) {
		r1 = obb1.getRadii[0] * absRot[0][i] + obb1.getRadii[1] * absRot[1][i] + obb1.getRadii[2] * absRot[2][i];
		r2 = obb2.getRadii[i];
		if (abs(t[0] * rot[0][i] + t[1] * rot[1][i] + t[2] * rot[2][i]) > (r1 + r2)) { return false; }
	}
	// Test axis L = 1[0] x 2[0]
	r1 = obb1.getRadii[1] * absRot[2][0] + obb1.getRadii[2] * absRot[1][0];
	r2 = obb2.getRadii[1] * absRot[0][2] + obb2.getRadii[2] * absRot[0][1];
	if (abs(t[2] * rot[1][0] - t[1] * rot[2][0]) > (r1 + r2)) { return false; }
	// Test axis L = 1[0] x 2[1]
	r1 = obb1.getRadii[1] * absRot[2][1] + obb1.getRadii[2] * absRot[1][1];
	r2 = obb2.getRadii[0] * absRot[0][2] + obb2.getRadii[2] * absRot[0][0];
	if (abs(t[2] * rot[1][1] - t[1] * rot[2][1]) > (r1 + r2)) { return false; }
	// Test axis L = 1[0] x 2[2]
	r1 = obb1.getRadii[1] * absRot[2][2] + obb1.getRadii[2] * absRot[1][2];
	r2 = obb2.getRadii[0] * absRot[0][1] + obb2.getRadii[1] * absRot[0][0];
	if (abs(t[2] * rot[1][2] - t[1] * rot[2][2]) > (r1 + r2)) { return false; }
	// Test axis L = 1[1] x 2[0]
	r1 = obb1.getRadii[0] * absRot[2][0] + obb1.getRadii[2] * absRot[0][0];
	r2 = obb2.getRadii[1] * absRot[1][2] + obb2.getRadii[2] * absRot[1][1];
	if (abs(t[0] * rot[2][0] - t[2] * rot[0][0]) > (r1 + r2)) { return false; }
	// Test axis L = 1[1] x 2[1]
	r1 = obb1.getRadii[0] * absRot[2][1] + obb1.getRadii[2] * absRot[0][1];
	r2 = obb2.getRadii[0] * absRot[1][2] + obb2.getRadii[2] * absRot[1][0];
	if (abs(t[0] * rot[2][1] - t[2] * rot[0][1]) > (r1 + r2)) { return false; }
	// Test axis L = 1[1] x 2[2]
	r1 = obb1.getRadii[0] * absRot[2][2] + obb1.getRadii[2] * absRot[0][2];
	r2 = obb2.getRadii[0] * absRot[1][1] + obb2.getRadii[1] * absRot[1][0];
	if (abs(t[0] * rot[2][2] - t[2] * rot[0][2]) > (r1 + r2)) { return false; }
	// Test axis L = 1[2] x 2[0]
	r1 = obb1.getRadii[0] * absRot[1][0] + obb1.getRadii[1] * absRot[0][0];
	r2 = obb2.getRadii[1] * absRot[2][2] + obb2.getRadii[2] * absRot[2][1];
	if (abs(t[1] * rot[0][0] - t[0] * rot[1][0]) > (r1 + r2)) { return false; }
	// Test axis L = 1[2] x 2[1]
	r1 = obb1.getRadii[0] * absRot[1][1] + obb1.getRadii[1] * absRot[0][1];
	r2 = obb2.getRadii[0] * absRot[2][2] + obb2.getRadii[2] * absRot[2][0];
	if (abs(t[1] * rot[0][1] - t[0] * rot[1][1]) > (r1 + r2)) { return false; }
	// Test axis L = 1[2] x 2[2]
	r1 = obb1.getRadii[0] * absRot[1][2] + obb1.getRadii[1] * absRot[0][2];
	r2 = obb2.getRadii[0] * absRot[2][1] + obb2.getRadii[1] * absRot[2][0];
	if (abs(t[1] * rot[0][2] - t[0] * rot[1][2]) > (r1 + r2)) { return false; }
	// If all tests failed
	return true;
}


// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 3.0f, 20.0f));

	// create environment ( large plane at y=-3)
	Mesh plane = Mesh::Mesh(Mesh::QUAD);
	//Mesh plane = Mesh::Mesh("resources/models/plane10.obj");
	plane.setShader(Shader("resources/shaders/physics.vert", "resources/shaders/transp.frag"));
	plane.scale(glm::vec3(20.0f, 20.0f, 20.0f));
	plane.translate(glm::vec3(0.0f, 0.0f, 0.0f));

	// rigid body set up
	RigidBody rb1 = RigidBody();

	// create sphere from obj
	//Mesh m1 = Mesh::Mesh("resources/models/sphere1.obj");

	// create cube from obj
	//Mesh m1 = Mesh::Mesh("resources/models/cube1.obj");

	// load triangle
	//Mesh m1 = Mesh::Mesh(Mesh::TRIANGLE);

	// load quad
	//Mesh m1 = Mesh::Mesh(Mesh::QUAD);

	// create cube
	Mesh m1 = Mesh::Mesh(Mesh::CUBE);

	rb1.setMesh(m1);
	Shader rbShader = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	rb1.getMesh().setShader(rbShader);
	//rb1.setBoxInvInertia();
	rb1.setMass(1.0f);
	rb1.translate(glm::vec3(5.0f, 3.0f, 0.0f));
	rb1.setVel(glm::vec3(-1.0f, 0.0f, 0.0f));
	rb1.setAngVel(glm::vec3(0.5f, 0.5f, 0.0f));
	rb1.setColl(Obb::Obb());

	// create cube2
	Mesh m2 = Mesh::Mesh(Mesh::CUBE);
	RigidBody rb2 = RigidBody();

	rb2.setMesh(m2);
	rb2.getMesh().setShader(rbShader);
	//rb1.setBoxInvInertia();
	rb2.setMass(1.0f);
	rb2.translate(glm::vec3(0.0f, 3.0f, 0.0f));
	rb2.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	rb2.setAngVel(glm::vec3(0.1f, -0.6f, 0.0f));
	rb2.setColl(Obb::Obb());

	Particle p1 = Particle::Particle();
	p1.setMesh(Mesh::Mesh(Mesh::MeshType::TRIANGLE));
	p1.translate(glm::vec3(-1.0f, 3.0f, 0.0f));
	p1.getMesh().setShader(rbShader);
	Particle p2 = Particle::Particle();
	p2.setMesh(Mesh::Mesh(Mesh::MeshType::TRIANGLE));
	p2.translate(glm::vec3(1.0f, 3.0f, 0.0f));
	p2.getMesh().setShader(rbShader);

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

				// integration (rotation)
				integrateRot(rb1, dt);
				integrateRot(rb2, dt);

				// check collisions


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
		app.draw(p1.getMesh());
		app.draw(p2.getMesh());

		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

