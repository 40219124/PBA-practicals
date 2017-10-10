#pragma once
// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"

// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Mesh.h"
#include "Body.h"
#include "Particle.h"

// time
GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;

struct ParticleS {
	Mesh particle;
	glm::vec3 velocity;
	float drag;
	float mass;
};

struct WindCone {
	glm::vec3 heightLimit;
	glm::vec3 origin;
	// Arc width based on the cosine of the angle between the vectors of the centre of the cone and its perimeter
	float arcWidth;
	float force;
};

glm::vec3 CalculateDrag(const float drag, const glm::vec3 &velocity) {
	glm::vec3 dragVector = -velocity;
	float velLength = sqrtf(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
	for (int i = 0; i < 3; ++i) {
		dragVector[i] = (dragVector[i] * drag * velLength) / 10.0f;
	}
	return dragVector;
}

void CreateApplication(Application &app, const glm::vec3 &cameraPos) {
	app.initRender();
	Application::camera.setCameraPosition(cameraPos);
}

Mesh CreatePlane(const float scale) {
	// create ground plane
	Mesh plane = Mesh::Mesh();
	// scale it up x5
	plane.scale(glm::vec3(scale, scale, scale));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));
	return plane;
}

Mesh CreateDefaultParticleMesh() {
	Mesh particle = Mesh::Mesh();
	particle.translate(glm::vec3(0.0f, 2.5f, 0.0f));
	particle.scale(glm::vec3(.1f, .1f, .1f));
	particle.rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	particle.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	return particle;
}

ParticleS CreateParticleS() {
	ParticleS particle;
	particle.particle = CreateDefaultParticleMesh();
	particle.drag = 0.001f;
	particle.mass = 0.01f;
	return particle;
}

std::vector<ParticleS> CreateParticleSVector(ParticleS particle, const int particleCount, const float power) {
	std::vector<ParticleS> particles = std::vector<ParticleS>();
	for (int i = 0; i < particleCount; ++i) {
		particle.velocity = glm::vec3((float)cos(M_PI * 2.0f * i / particleCount) * power, 8.0f, (float)sin(M_PI * 2.0f * i / particleCount) * power);
		particles.push_back(particle);
	}
	return particles;
}

void CreateParticleVector(std::vector<Particle> &particles, Particle particle, const int particleCount, const float power) {
	for (int i = 0; i < particleCount; ++i) {
		particle.setVel(glm::vec3((float)cos(M_PI * 2.0f * i / particleCount) * power, 1.0f, (float)sin(M_PI * 2.0f * i / particleCount) * power));
		particles.push_back(particle);
	}
}

void CreateCubeVectors(const float cubeWidth, glm::vec3 &cubeSize, glm::vec3 &cubeBL) {
	cubeSize = glm::vec3(cubeWidth, cubeWidth, cubeWidth);
	cubeBL = glm::vec3(-cubeWidth / 2.0f, 0.0f, -cubeWidth / 2.0f);
}

void IntegrationCollisionDetection(Particle &particle, const glm::vec3 &cubeBL, const glm::vec3 &cubeSize) {
	for (int i = 0; i < 3; ++i) {
		if (particle.getPos()[i] <= cubeBL[i]) {
			glm::vec3 vel = particle.getVel();
			vel[i] = abs(vel[i]);
			particle.setVel(vel);
			glm::vec3 pos = particle.getPos();
			pos[i] = cubeBL[i] + (cubeBL[i] - pos[i]);
			particle.setPos(pos);
		}
		else if (particle.getPos()[i] >= cubeBL[i] + cubeSize[i]) {
			glm::vec3 vel = particle.getVel();
			vel[i] = -abs(vel[i]);
			particle.setVel(vel);
			glm::vec3 pos = particle.getPos();
			pos[i] = (cubeBL[i] + cubeSize[i]) + ((cubeBL[i] + cubeSize[i]) - particle.getPos()[i]);
			particle.setPos(pos);
		}
	}
}

void CollisionDetection(Particle &particle, const glm::vec3 &cubeBL, const glm::vec3 &cubeSize) {
	for (int i = 0; i < 3; ++i) {
		if (particle.getPos()[i] <= cubeBL[i]) {
			glm::vec3 vel = particle.getVel() * 0.85f;
			vel[i] = abs(vel[i]) * 0.9f;
			particle.setVel(vel);
			glm::vec3 pos = particle.getPos();
			pos[i] = cubeBL[i] + (cubeBL[i] - pos[i]);
			particle.setPos(pos);
		}
		else if (particle.getPos()[i] >= cubeBL[i] + cubeSize[i]) {
			glm::vec3 vel = particle.getVel() * 0.85f;
			vel[i] = -abs(vel[i]) * 0.9f;
			particle.setVel(vel);
			glm::vec3 pos = particle.getPos();
			pos[i] = (cubeBL[i] + cubeSize[i]) + ((cubeBL[i] + cubeSize[i]) - particle.getPos()[i]);
			particle.setPos(pos);
		}
	}
}

float Magnitude(const glm::vec3 &vec) {
	return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

glm::vec3 NormaliseVector(const glm::vec3 &vec) {
	return vec / Magnitude(vec);
}

float NormalisedDotProduct(glm::vec3 vectorA, glm::vec3 vectorB) {
	vectorA = NormaliseVector(vectorA);
	vectorB = NormaliseVector(vectorB);
	return vectorA.x * vectorB.x + vectorA.y * vectorB.y + vectorA.z * vectorB.z;
}

glm::vec3 ParticleAcceleration(const WindCone &cone, const Particle &particle) {
	glm::vec3 force = glm::vec3(0.0f);
	glm::vec3 pos = (glm::vec3)particle.getTranslate()[3];
	float dotProduct = NormalisedDotProduct(cone.heightLimit - cone.origin, pos - cone.origin);
	if (dotProduct > cone.arcWidth && Magnitude(cone.heightLimit - cone.origin) > Magnitude(pos - cone.origin)) {
		force += cone.force * NormaliseVector(pos - cone.origin);
		force *= 1.0f - (1.0f - dotProduct) / (1.0f - cone.arcWidth);
		force *= 1.0f - Magnitude(pos - cone.origin) / Magnitude(cone.heightLimit - cone.origin);
	}
	glm::vec3 acc = glm::vec3(0.0f, -9.8f, 0.0f);
	acc += force / particle.getMass();
	return acc;
}

void BoxDemo() {
	// create application
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 5.0f, 20.0f));

	Mesh plane = CreatePlane(5.0f);

	ParticleS particle = CreateParticleS();
	int particleCount = 38;// 3;
	std::vector<ParticleS> particles = CreateParticleSVector(particle, particleCount, 5.0);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	glm::vec3 velocity1 = glm::vec3(5.0f, 18.0f, 9.0f);


	// time
	GLfloat firstFrame = (GLfloat)glfwGetTime();

	double totalTime = 0.0;
	double fixedDeltaTime = 0.01;
	double startTime = glfwGetTime();
	double accumulator = 0.0;
	double lastFrameTime = startTime;
	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		// Set frame time
		GLfloat currentFrame = (GLfloat)glfwGetTime() - firstFrame;
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = currentFrameTime - lastFrameTime;

		accumulator += frameDeltaTime;
		// the animation can be sped up or slowed down by multiplying currentFrame by a factor.
		currentFrame *= 1.5f;
		deltaTime = currentFrame - lastFrame;

		/*
		**	INTERACTION
		*/
		// Manage interaction
		app.doMovement(deltaTime);

		while (accumulator >= fixedDeltaTime) {
			/*
			**	SIMULATION
			*/
			for (int i = 0; i < particleCount; ++i) {
				glm::vec3 acceleration = CalculateDrag(particles[i].drag, particles[i].velocity) / particles[i].mass;
				acceleration += aGravity;
				particles[i].velocity = particles[i].velocity + acceleration * fixedDeltaTime;
				particles[i].particle.translate(particles[i].velocity * fixedDeltaTime);
				glm::vec3 particlePos = glm::vec3(particles[i].particle.getTranslate()[3]);
				for (int j = 0; j < 3; ++j) {
					if (particlePos[j] < cubeBL[j]) {
						particles[i].velocity = particles[i].velocity * 0.9;
						particles[i].velocity[j] = abs(particles[i].velocity[j]);
						particlePos[j] = cubeBL[j] + (cubeBL[j] - particlePos[j]);
						particles[i].particle.setPos(particlePos);
					}
					else if (particlePos[j] > cubeBL[j] + cubeSize[j]) {
						particles[i].velocity = particles[i].velocity * 0.9;
						particles[i].velocity[j] = -abs(particles[i].velocity[j]);
						particlePos[j] = (cubeBL[j] + cubeSize[j]) - (particlePos[j] - (cubeBL[j] + cubeSize[j]));
						particles[i].particle.setPos(particlePos);
					}
				}
			}
			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}
		/*
		**	RENDER
		*/
		// clear buffer
		app.clear();
		// draw groud plane
		app.draw(plane);
		// draw particles
		for (int i = 0; i < particleCount; ++i) {
			app.draw(particles[i].particle);
		}

		app.display();
		lastFrame = currentFrame;
		lastFrameTime = currentFrameTime;
		std::cout << 1.0f / deltaTime << std::endl;
	}

	app.terminate();
}

void IntegrateDemo() {
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 5.0f, 20.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);

	Particle staticParticle = Particle::Particle();
	staticParticle.setPos(glm::vec3(-1.5f, 2.0f, 0.0f));
	staticParticle.setVel(glm::vec3(0.0f));
	staticParticle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	Particle feParticle = Particle::Particle();
	feParticle.setPos(glm::vec3(0.0f, 2.0f, 0.0f));
	feParticle.setVel(glm::vec3(0.0f));
	feParticle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	Particle siParticle = Particle::Particle();
	siParticle.setPos(glm::vec3(1.5f, 2.0f, 0.0f));
	siParticle.setVel(glm::vec3(0.0f));
	siParticle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	double totalTime = 0.0;
	double fixedDeltaTime = 0.005;
	double startTime = glfwGetTime();
	double accumulator = 0.0;
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = currentFrameTime - lastFrameTime;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime);

		while (accumulator >= fixedDeltaTime) {
			feParticle.translate(feParticle.getVel() * fixedDeltaTime);
			feParticle.setVel(feParticle.getVel() + aGravity * fixedDeltaTime);
			IntegrationCollisionDetection(feParticle, cubeBL, cubeSize);

			siParticle.setVel(siParticle.getVel() + aGravity * fixedDeltaTime);
			siParticle.translate(siParticle.getVel() * fixedDeltaTime);
			IntegrationCollisionDetection(siParticle, cubeBL, cubeSize);

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();

		app.draw(plane);

		app.draw(staticParticle.getMesh());
		app.draw(feParticle.getMesh());
		app.draw(siParticle.getMesh());

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << 1.0f / frameDeltaTime << std::endl;
	}
}

void WindDemo() {

	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 10.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);

	bool funWind = true;
	WindCone cone;
	cone.arcWidth = cosf((float)M_PI / 6.0f);
	if (cone.arcWidth == 1.0f) {
		cone.arcWidth = 0.99f;
	}
	cone.force = 1.5f;
	cone.heightLimit = glm::vec3(0.0f, 3.0f, 0.0f);
	cone.origin = glm::vec3(0.0f, -2.0f, 0.0f);
	if (funWind) {
		cone.arcWidth = cosf((float)M_PI / 3.0f);
	}

	Particle particle = Particle::Particle();
	particle.setPos(glm::vec3(-1.0f, 1.0f, -1.0f));
	particle.setVel(glm::vec3(0.0f));
	particle.setMass(0.01f);
	particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	int particleCount = 100;
	std::vector<Particle> particles;
	CreateParticleVector(particles, particle, particleCount, 5.0f);

	double totalTime = 0.0;
	double fixedDeltaTime = 0.005;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = currentFrameTime - lastFrameTime;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime);

		while (accumulator >= fixedDeltaTime) {
			if (funWind) {
				cone.origin = glm::vec3(cosf((float)M_PI * (float)totalTime / 5.0f) * 3.0f, cone.origin.y, sinf((float)M_PI * (float)totalTime / 5.0f) * 3.0f);
				cone.heightLimit = glm::vec3(cosf((float)M_PI * (float)totalTime / 5.0f) * 3.0f, cone.heightLimit.y, sinf((float)M_PI * (float)totalTime / 5.0f) * 3.0f);
			}

			for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
				particles[pIndex].setVel(particles[pIndex].getVel() + ParticleAcceleration(cone, particles[pIndex]) * fixedDeltaTime);
				particles[pIndex].translate(particles[pIndex].getVel() * fixedDeltaTime);
				CollisionDetection(particles[pIndex], cubeBL, cubeSize);
			}

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();

		app.draw(plane);

		for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
			app.draw(particles[pIndex].getMesh());
		}

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << (1.0f / frameDeltaTime < 60.0f ? 1.0f/frameDeltaTime : 0.0f) << std::endl;
	}
}

void ChainDemo() {

	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 10.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	Gravity grav = Gravity(glm::vec3(0.0f, -9.8f, 0.0f));
	Drag drag = Drag();

	Particle particle = Particle::Particle();
	particle.setPos(glm::vec3(-1.0f, 3.0f, -1.0f));
	particle.setVel(glm::vec3(0.0f));
	particle.setMass(0.01f);
	particle.addForce(&grav);
	particle.addForce(&drag);
	particle.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	int particleCount = 100;
	std::vector<Particle> particles;
	CreateParticleVector(particles, particle, particleCount, 5.0f);

	double totalTime = 0.0;
	double fixedDeltaTime = 0.005;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = currentFrameTime - lastFrameTime;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime);

		while (accumulator >= fixedDeltaTime) {

			for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
				particles[pIndex].setAcc(particles[pIndex].applyForces(particles[pIndex].getPos(), particles[pIndex].getVel(), totalTime, fixedDeltaTime));
				particles[pIndex].setVel(particles[pIndex].getVel() + particles[pIndex].getAcc() * fixedDeltaTime);
				particles[pIndex].translate(particles[pIndex].getVel() * fixedDeltaTime);
				CollisionDetection(particles[pIndex], cubeBL, cubeSize);
			}

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();

		app.draw(plane);

		for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
			app.draw(particles[pIndex].getMesh());
		}

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << (1.0f / frameDeltaTime < 60.0f ? 1.0f / frameDeltaTime : 0.0f) << std::endl;
	}
}

// main function
int main()
{
	int demo = 3;
	switch (demo) {
	case 0:
		BoxDemo();
		break;
	case 1:
		IntegrateDemo();
		break;
	case 2:
		WindDemo();
		break;
	case 3:
		ChainDemo();
		break;
	default:
		BoxDemo();
		break;
	}

	//// create particle
	//Mesh particle1 = Mesh::Mesh();
	////scale it down (x.1), translate it up by 2.5 and rotate it by 90 degrees around the x axis
	//particle1.translate(glm::vec3(0.0f, 2.5f, 0.0f));
	//particle1.scale(glm::vec3(.1f, .1f, .1f));
	//particle1.rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	//particle1.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));

	return EXIT_SUCCESS;
}