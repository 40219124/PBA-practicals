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
	plane.translate(glm::vec3(0.0f, -3.0f, 0.0f));

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
	rb1.translate(glm::vec3(5.0f, 0.1f, 0.0f));
	rb1.setVel(glm::vec3(-1.0f, 0.0f, 0.0f));
	rb1.setAngVel(glm::vec3(0.5f, .5f, 0.0f));
	// add forces to Rigid body
	//rb1.addForce(&g);

	// create cube2
	Mesh m2 = Mesh::Mesh(Mesh::CUBE);
	RigidBody rb2 = RigidBody();

	rb2.setMesh(m2);
	rb2.getMesh().setShader(rbShader);
	//rb1.setBoxInvInertia();
	rb2.setMass(1.0f);
	rb2.translate(glm::vec3(-5.0f, 0.1f, 0.0f));
	rb2.setVel(glm::vec3(1.0f, 0.0f, 0.0f));
	rb2.setAngVel(glm::vec3(0.1f, -0.6f, 0.0f));







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

		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

