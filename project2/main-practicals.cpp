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
#include "RigidBody.h"

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
	Mesh plane = Mesh::Mesh(Mesh::QUAD);
	// scale it up x5
	plane.scale(glm::vec3(scale, scale, scale));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));
	return plane;
}

Mesh CreateDefaultParticleMesh() {
	Mesh particle = Mesh::Mesh(Mesh::QUAD);
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

	Mesh plane = CreatePlane(2.5f);

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
		std::cout << (1.0f / frameDeltaTime < 60.0f ? 1.0f / frameDeltaTime : 0.0f) << std::endl;
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
	float particleMass = 0.1f;
	Gravity grav = Gravity(glm::vec3(0.0f, -9.8f, 0.0f) * particleMass);

	float spring = 9.5f;
	float damp = 7.0f;
	float rest = 0.5f;

	int particleCount = 5;
	std::vector<Particle> particles(particleCount);

	for (int i = 0; i < particleCount; ++i) {
		particles[i] = (Particle::Particle());
		particles[i].getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));
		particles[i].setVel(glm::vec3((float)cos(M_PI * 2.0f * i / particleCount) * 5.0f, 1.0f, (float)sin(M_PI * 2.0f * i / particleCount) * 5.0f));
		particles[i].setPos(glm::vec3(0.0f, 4.0f + i / particleCount, 0.0f));
		particles[i].setMass(particleMass);
		particles[i].addForce(&grav);
		if (i > 0) {
			Hooke *hooke = new Hooke(&particles[i - 1], &particles[i], spring, damp, rest);
			particles[i - 1].addForce(hooke);
			particles[i].addForce(hooke);
		}
	}
	particles[0].setVel(glm::vec3(0.0f));

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 1000.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		while (accumulator >= fixedDeltaTime) {

			for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
				particles[pIndex].setAcc(particles[pIndex].applyForces(particles[pIndex].getPos(), particles[pIndex].getVel(), (float)totalTime, (float)fixedDeltaTime));
			}
			particles[0].setAcc(glm::vec3(0.0f));
			for (int pIndex = 1; pIndex < particleCount; ++pIndex) {
				particles[pIndex].setVel(particles[pIndex].getVel() + particles[pIndex].getAcc() * fixedDeltaTime);
				particles[pIndex].translate(particles[pIndex].getVel() * fixedDeltaTime);
			}

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();

		for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
			app.draw(particles[pIndex].getMesh());
		}

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << (1.0f / frameDeltaTime < 60.0f ? 1.0f / frameDeltaTime : 0.0f) << std::endl;
	}
}

void UDemo() {

	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 10.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	float particleMass = 0.1f;
	Gravity grav = Gravity(glm::vec3(0.0f, -9.8f, 0.0f) * particleMass);

	float spring = 14.5f;
	float damp = 8.0f;
	float rest = 0.5f;

	int particleCount = 10;
	std::vector<Particle> particles(particleCount);

	for (int i = 0; i < particleCount; ++i) {
		particles[i] = (Particle::Particle());
		particles[i].getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));
		particles[i].setVel(glm::vec3(0.0f));
		particles[i].setPos(glm::vec3(-2.0f + 4.0f * i / (particleCount - 1), 4.0f, 0.0f));
		particles[i].setMass(particleMass);
		particles[i].addForce(&grav);
		if (i > 0) {
			Hooke *hooke = new Hooke(&particles[i - 1], &particles[i], spring, damp, rest);
			particles[i - 1].addForce(hooke);
			particles[i].addForce(hooke);
		}
	}

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 1000.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		while (accumulator >= fixedDeltaTime) {

			for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
				particles[pIndex].setAcc(particles[pIndex].applyForces(particles[pIndex].getPos(), particles[pIndex].getVel(), (float)totalTime, (float)fixedDeltaTime));
			}
			particles[0].setAcc(glm::vec3(0.0f));
			particles[particleCount - 1].setAcc(glm::vec3(0.0f));
			for (int pIndex = 1; pIndex < particleCount - 1; ++pIndex) {
				particles[pIndex].setVel(particles[pIndex].getVel() + particles[pIndex].getAcc() * fixedDeltaTime);
				particles[pIndex].translate(particles[pIndex].getVel() * fixedDeltaTime);
			}

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();

		for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
			app.draw(particles[pIndex].getMesh());
		}

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << (1.0f / frameDeltaTime < 60.0f ? 1.0f / frameDeltaTime : 0.0f) << std::endl;
	}
}

void U2Demo() {

	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 10.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	float particleMass = 0.1f;
	Gravity grav = Gravity(glm::vec3(0.0f, -9.8f, 0.0f) * particleMass);

	float spring = 14.5f;
	float damp = 8.0f;
	float rest = 0.5f;

	int particleCount = 10;
	std::vector<Particle> particles(particleCount);

	for (int i = 0; i < particleCount; ++i) {
		particles[i] = (Particle::Particle());
		particles[i].getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));
		particles[i].setVel(glm::vec3(0.0f));
		particles[i].setPos(glm::vec3(-2.0f + 4.0f * i / (particleCount - 1), 2.0f, 0.0f));
		particles[i].setMass(particleMass);
		particles[i].addForce(&grav);
		if (i > 0) {
			Hooke *hooke = new Hooke(&particles[i - 1], &particles[i], spring, damp, rest);
			particles[i - 1].addForce(hooke);
			particles[i].addForce(hooke);
		}
	}

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 1000.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		while (accumulator >= fixedDeltaTime) {

			for (int pIndex = 0; pIndex < particleCount; ++pIndex) {
				particles[pIndex].setAcc(particles[pIndex].applyForces(particles[pIndex].getPos(), particles[pIndex].getVel(), (float)totalTime, (float)fixedDeltaTime));
			}
			particles[0].setAcc(glm::vec3(0.0f));
			particles[particleCount - 1].setAcc(glm::vec3(0.0f));
			for (int pIndex = 1; pIndex < particleCount - 1; ++pIndex) {
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

void ClothDemo() {

	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 10.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	float particleMass = 0.1f;
	Gravity grav = Gravity(glm::vec3(0.0f, -9.8f, 0.0f) * particleMass);

	const int sideLength = 10;
	Particle cloth[sideLength][sideLength];
	glm::vec3 clothStart = glm::vec3(-2.0, 3.0f, -2.0f);
	glm::vec3 clothDim = glm::vec3(4.0f, 0.0f, 4.0f);

	float spring = 12.0f;
	float damp = 2.0f;
	float rest = clothDim.x / ((float)sideLength * 1.25f);

	for (int x = 0; x < sideLength; ++x) {
		for (int z = 0; z < sideLength; ++z) {
			cloth[x][z] = Particle::Particle();
			cloth[x][z].getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));
			cloth[x][z].setPos(glm::vec3(clothStart.x + clothDim.x * x / (sideLength - 1), clothStart.y, clothStart.z + clothDim.z * z / (sideLength - 1)));
			cloth[x][z].setMass(particleMass);
			cloth[x][z].addForce(&grav);
			if (z > 0) {
				Hooke *hooke = new Hooke(&cloth[x][z - 1], &cloth[x][z], spring, damp, rest);
				cloth[x][z - 1].addForce(hooke);
				cloth[x][z].addForce(hooke);
			}
			if (x > 0) {
				Hooke *hooke = new Hooke(&cloth[x - 1][z], &cloth[x][z], spring, damp, rest);
				cloth[x - 1][z].addForce(hooke);
				cloth[x][z].addForce(hooke);
			}
		}
	}

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 200.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		while (accumulator >= fixedDeltaTime) {

			for (int x = 0; x < sideLength; ++x) {
				for (int z = 0; z < sideLength; ++z) {
					cloth[x][z].setAcc(cloth[x][z].applyForces(cloth[x][z].getPos(), cloth[x][z].getVel(), (float)totalTime, (float)fixedDeltaTime));
					if (x == 0 && (z == 0 || z == sideLength - 1)) {
						cloth[x][z].setAcc(glm::vec3(0.0f));
					}
					else if (x == sideLength - 1 && (z == 0 || z == sideLength - 1)) {
						cloth[x][z].setAcc(glm::vec3(0.0f));
					}
				}
			}

			for (int x = 0; x < sideLength; ++x) {
				for (int z = 0; z < sideLength; ++z) {
					cloth[x][z].setVel(cloth[x][z].getVel() + cloth[x][z].getAcc() * fixedDeltaTime);
					cloth[x][z].translate(cloth[x][z].getVel() * fixedDeltaTime);
					CollisionDetection(cloth[x][z], cubeBL, cubeSize);
				}
			}

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();
		app.draw(plane);

		for (int x = 0; x < sideLength; ++x) {
			for (int z = 0; z < sideLength; ++z) {
				app.draw(cloth[x][z].getMesh());
			}
		}

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << (1.0f / frameDeltaTime < 60.0f ? 1.0f / frameDeltaTime : 0.0f) << std::endl;
	}
}

void FlagDemo() {

	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 10.0f));

	Mesh plane = CreatePlane(5.0f);

	glm::vec3 cubeSize;
	glm::vec3 cubeBL;
	CreateCubeVectors(5.0f, cubeSize, cubeBL);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	float particleMass = 0.1f;
	Gravity grav = Gravity(aGravity * particleMass);

	glm::vec3 windSpeed = glm::vec3(0.0f, 0.0f, -3.0f);
	glm::vec3 windSource = glm::vec3(0.0f, cos(2.0f * M_PI * 0.0f), sin(2.0f * M_PI * 0.0f));
	glm::vec3 windEnd = -windSource;
	Particle wiSoP = Particle::Particle();
	wiSoP.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	Particle wiEnP = Particle::Particle();
	wiEnP.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_red.frag"));

	const int sideLength = 10;
	Particle cloth[sideLength][sideLength];
	glm::vec3 clothStart = glm::vec3(-2.0, 3.0f, -4.0f);
	glm::vec3 clothDim = glm::vec3(4.0f, 0.0f, 4.0f);

	float spring = 600.0f;
	float damp = 1.0f;
	float rest = clothDim.x / ((float)sideLength);// *1.25f);

	for (int x = 0; x < sideLength; ++x) {
		for (int z = 0; z < sideLength; ++z) {
			cloth[x][z] = Particle::Particle();
			cloth[x][z].getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));
			cloth[x][z].setPos(glm::vec3(clothStart.x + clothDim.x * x / (sideLength - 1), clothStart.y, clothStart.z + clothDim.z * z / (sideLength - 1)));
			cloth[x][z].setMass(particleMass);
			cloth[x][z].addForce(&grav);
			if (z > 0) {
				Hooke *hooke = new Hooke(&cloth[x][z - 1], &cloth[x][z], spring, damp, rest);
				cloth[x][z - 1].addForce(hooke);
				cloth[x][z].addForce(hooke);
			}
			if (x > 0) {
				Hooke *hooke = new Hooke(&cloth[x - 1][z], &cloth[x][z], spring, damp, rest);
				cloth[x - 1][z].addForce(hooke);
				cloth[x][z].addForce(hooke);
			}
			if (x > 0) {
				Wind *wind = new Wind(&cloth[x][z], &cloth[x - 1][z], &cloth[x - 1][z + 1], &windSpeed);
				cloth[x][z].addForce(wind);
				cloth[x - 1][z].addForce(wind);
				cloth[x - 1][z + 1].addForce(wind);
				if (z > 0) {
					wind = new Wind(&cloth[x][z], &cloth[x][z - 1], &cloth[x - 1][z], &windSpeed);
					cloth[x][z].addForce(wind);
					cloth[x][z - 1].addForce(wind);
					cloth[x - 1][z].addForce(wind);
				}
			}
		}
	}

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 500.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	int frameCount = 1;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		accumulator += frameDeltaTime;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		while (accumulator >= fixedDeltaTime) {

			windSource = glm::vec3(0.0f, cos(M_PI * totalTime / 10.0f), sin(M_PI * totalTime / 10.0f));
			windEnd = -windSource;
			windSpeed = (windEnd - windSource) * 4.0f;
			wiSoP.setPos(windSource);
			wiSoP.setPos(0, clothStart.x);
			wiSoP.setPos(1, wiSoP.getPos().y + clothStart.y);
			wiEnP.setPos(windEnd);
			wiEnP.setPos(0, clothStart.x);
			wiEnP.setPos(1, wiEnP.getPos().y + clothStart.y);

			for (int x = 0; x < sideLength; ++x) {
				for (int z = 0; z < sideLength; ++z) {
					if (z == sideLength - 1 && (x == 0 || x == sideLength - 1)) {
						cloth[x][z].setAcc(glm::vec3(0.0f));
					}
					else {
						cloth[x][z].setAcc(cloth[x][z].applyForces(cloth[x][z].getPos(), cloth[x][z].getVel(), (float)totalTime, (float)fixedDeltaTime));
					}
				}
			}

			for (int x = 0; x < sideLength; ++x) {
				for (int z = 0; z < sideLength; ++z) {
					cloth[x][z].setVel(cloth[x][z].getVel() + cloth[x][z].getAcc() * fixedDeltaTime);
					cloth[x][z].translate(cloth[x][z].getVel() * fixedDeltaTime);
					int i = 1;
					if (cloth[x][z].getPos()[i] <= cubeBL[i]) {
						glm::vec3 vel = cloth[x][z].getVel() * 0.85f;
						vel[i] = abs(vel[i]) * 0.9f;
						cloth[x][z].setVel(vel);
						glm::vec3 pos = cloth[x][z].getPos();
						pos[i] = cubeBL[i] + (cubeBL[i] - pos[i]);
						cloth[x][z].setPos(pos);
					}
				}
			}

			accumulator -= fixedDeltaTime;
			totalTime += fixedDeltaTime;
		}

		app.clear();
		app.draw(plane);
		app.draw(wiSoP.getMesh());
		app.draw(wiEnP.getMesh());

		for (int x = 0; x < sideLength; ++x) {
			for (int z = 0; z < sideLength; ++z) {
				app.draw(cloth[x][z].getMesh());
			}
		}

		app.display();

		lastFrameTime = currentFrameTime;
		std::cout << (1.0f / ((currentFrameTime - startTime) / frameCount)) << std::endl;
		frameCount++;
	}
}

void ApplyImpulse(RigidBody &rb, const glm::vec3 &impLoc, const glm::vec3 &impDir) {
	glm::vec3 deltaV = impDir / rb.getMass();
	glm::vec3 deltaW = rb.getInvInertia() * (glm::cross(impLoc, impDir));
	rb.setVel(rb.getVel() + deltaV);
	rb.setAngVel(rb.getAngVel() + deltaW);
}

void FirstRB() {
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 20.0f));

	Mesh plane = CreatePlane(10.0f);
	plane.setPos(glm::vec3(0.0f));
	glm::vec3 nPlane = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	Gravity grav = Gravity::Gravity(aGravity);

	Shader shader = Shader("resources/shader/core.vert", "resources/shaders/core_green.frag");

	RigidBody cube = RigidBody();
	Mesh cMesh = Mesh::Mesh(Mesh::CUBE);
	cube.setMesh(cMesh);
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));

	cube.translate(glm::vec3(0.0f, 5.0f, 0.0f));
	cube.setVel(glm::vec3(1.0f, 0.0f, 0.0f));
	cube.setAngVel(glm::vec3(0.1f, 0.5f, 0.0f));
	cube.setAngAccl(glm::vec3(0.0f, 0.0f, 0.0f));

	cube.setMass(1.0f);
	cube.setCor(0.90f);
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));

	cube.addForce(&grav);

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 500.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	bool pause = false;

	glm::vec3 impLoc = glm::vec3(1.0f, 03.5f, 0.0f);
	glm::vec3 impDir = glm::vec3(-3.0f, 0.0f, 0.0f);
	bool hit = false;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		if (!pause) {
			accumulator += frameDeltaTime;

			while (accumulator >= fixedDeltaTime) {
				cube.setAcc(cube.applyForces(cube.getPos(), cube.getVel(), (float)totalTime, deltaTime));

				if (totalTime >= 2.0f && !hit) {
					//ApplyImpulse(cube, impLoc, impDir);
					hit = true;
				}
				else {
					cube.setVel(cube.getVel() + cube.getAcc() * fixedDeltaTime);
					cube.setAngVel(cube.getAngVel() + cube.getAngAcc() * fixedDeltaTime);
				}

				cube.translate(cube.getVel() * fixedDeltaTime);

				glm::mat3 angVelSkew = glm::matrixCross3(cube.getAngVel());
				glm::mat3 rot = glm::mat3(cube.getRotate());
				rot += angVelSkew * rot * fixedDeltaTime;
				rot = glm::orthonormalize(rot);
				cube.setRotate(glm::mat4(rot));

				std::vector<Vertex> verts = cube.getMesh().getVertices();
				glm::vec3 vertAv = glm::vec3(0.0f);
				float colCount = 0.0f;
				glm::mat4 m = cube.getMesh().getModel();
				for (int vCount = 0; vCount < (int)verts.size(); ++vCount) {
					glm::vec4 vWorld = m * glm::vec4(verts[vCount].getCoord(), 1.0f);
					vWorld = vWorld / vWorld.w;
					if (vWorld.y < 0.0f && cube.getVel().y < 0.0f) {
						vertAv += glm::vec3(vWorld);
						colCount++;
					}
				}
				if (vertAv != glm::vec3(0.0f)) {
					vertAv = vertAv / colCount;
					cube.translate(glm::vec3(0.0f, 0.0f - vertAv.y, 0.0f));
					vertAv = vertAv - cube.getPos();
					// jr = -(1+e)vr.n / m-1 +n.(I(r1*n))*r1
					glm::vec3 v1 = -(cube.getVel() + glm::cross(cube.getAngVel(), glm::vec3(vertAv)));
					float numer = -(1 + cube.getCor()) * glm::dot(-v1, nPlane);
					float denom = (1 / cube.getMass()) + glm::dot(nPlane, glm::cross((glm::mat3(cube.getRotate()) * cube.getInvInertia() * glm::transpose(glm::mat3(cube.getRotate()))) * glm::cross(glm::vec3(vertAv), nPlane), glm::vec3(vertAv)));
					float j = numer / denom;

					glm::vec3 vNew = cube.getVel() + nPlane * j / cube.getMass();
					glm::vec3 wNew = cube.getAngVel() + j * cube.getInvInertia() * glm::cross(glm::vec3(vertAv), nPlane);

					cube.setVel(vNew);
					cube.setAngVel(wNew);
				}

				if (totalTime < 2.0f) {
					cube.setAngAccl(cube.getAngAcc() - cube.getAngAcc() * (fixedDeltaTime) / (2.0f - totalTime));
				}

				accumulator -= fixedDeltaTime;
				totalTime += fixedDeltaTime;
			}
		}

		app.clear();
		app.draw(plane);
		app.draw(cube.getMesh());

		app.display();

		lastFrameTime = currentFrameTime;

		static int frameCount = 1;
		std::cout << (1.0f / ((currentFrameTime - startTime) / frameCount)) << std::endl;
		frameCount++;
	}
}

void ImpDemo() {
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(2.0f, 4.0f, 20.0f));

	Shader shader = Shader("resources/shader/core.vert", "resources/shaders/core_green.frag");

	RigidBody cube = RigidBody();
	Mesh cMesh = Mesh::Mesh(Mesh::CUBE);
	cube.setMesh(cMesh);
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));

	cube.translate(glm::vec3(0.0f, 5.0f, 0.0f));
	cube.setVel(glm::vec3(2.0f, 0.0f, 0.0f));

	cube.setMass(2.0f);
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));

	std::cout << glm::to_string(cube.getInvInertia()) << std::endl;

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 500.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	bool pause = false;

	glm::vec3 impLoc = glm::vec3(1.0f, -1.0f, 0.0f);
	glm::vec3 impDir = glm::vec3(-4.0f, 0.0f, 0.0f);
	bool hit = false;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		if (!pause) {
			accumulator += frameDeltaTime;

			while (accumulator >= fixedDeltaTime) {
				if (totalTime >= 2.0f && !hit) {
					ApplyImpulse(cube, impLoc, impDir);
					hit = true;
				}
				else {
					cube.setVel(cube.getVel() + cube.getAcc() * fixedDeltaTime);
					cube.setAngVel(cube.getAngVel() + cube.getAngAcc() * fixedDeltaTime);
				}

				cube.translate(cube.getVel() * fixedDeltaTime);

				glm::mat3 angVelSkew = glm::matrixCross3(cube.getAngVel());
				glm::mat3 rot = glm::mat3(cube.getRotate());
				rot += angVelSkew * rot * fixedDeltaTime;
				rot = glm::orthonormalize(rot);
				cube.setRotate(glm::mat4(rot));

				accumulator -= fixedDeltaTime;
				totalTime += fixedDeltaTime;
			}
		}

		app.clear();
		app.draw(cube.getMesh());

		app.display();

		lastFrameTime = currentFrameTime;
	}
}

void CollDetDemo() {
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 20.0f));

	Mesh plane = CreatePlane(10.0f);
	plane.setPos(glm::vec3(0.0f));
	glm::vec3 nPlane = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	Gravity grav = Gravity::Gravity(aGravity);

	Shader shader = Shader("resources/shader/core.vert", "resources/shaders/core_green.frag");

	RigidBody cube = RigidBody();
	Mesh cMesh = Mesh::Mesh(Mesh::CUBE);
	cube.setMesh(cMesh);
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));

	cube.translate(glm::vec3(0.0f, 5.0f, 0.0f));
	cube.setVel(glm::vec3(1.0f, 0.0f, 0.0f));
	cube.setAngVel(glm::vec3(0.1f, 0.5f, 0.0f));

	cube.setMass(1.0f);
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));

	cube.addForce(&grav);

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 500.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	bool pause = false;

	glm::vec3 impLoc = glm::vec3(1.0f, 03.5f, 0.0f);
	glm::vec3 impDir = glm::vec3(-3.0f, 0.0f, 0.0f);
	bool hit = false;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		if (!pause) {
			accumulator += frameDeltaTime;

			while (accumulator >= fixedDeltaTime) {
				cube.setAcc(cube.applyForces(cube.getPos(), cube.getVel(), (float)totalTime, deltaTime));

				cube.setVel(cube.getVel() + cube.getAcc() * fixedDeltaTime);
				cube.setAngVel(cube.getAngVel() + cube.getAngAcc() * fixedDeltaTime);

				cube.translate(cube.getVel() * fixedDeltaTime);

				glm::mat3 angVelSkew = glm::matrixCross3(cube.getAngVel());
				glm::mat3 rot = glm::mat3(cube.getRotate());
				rot += angVelSkew * rot * fixedDeltaTime;
				rot = glm::orthonormalize(rot);
				cube.setRotate(glm::mat4(rot));

				std::vector<Vertex> verts = cube.getMesh().getVertices();
				glm::vec3 vertAv = glm::vec3(0.0f);
				int colCount = 0;
				glm::mat4 m = cube.getMesh().getModel();
				for (int vCount = 0; vCount < (int)verts.size(); ++vCount) {
					glm::vec4 vWorld = m * glm::vec4(verts[vCount].getCoord(), 1.0f);
					vWorld = vWorld / vWorld.w;
					if (vWorld.y < 0.0f) {
						std::cout << "vertex: " << glm::to_string(glm::vec3(vWorld)) << std::endl;
						vertAv += glm::vec3(vWorld);
						colCount++;
					}
				}
				if (colCount > 0) {
					pause = true;
					std::cout << "average: " << glm::to_string(vertAv / (float)colCount) << std::endl;
				}

				if (totalTime < 2.0f) {
					cube.setAngAccl(cube.getAngAcc() - cube.getAngAcc() * (fixedDeltaTime) / (2.0f - totalTime));
				}

				accumulator -= fixedDeltaTime;
				totalTime += fixedDeltaTime;
			}
		}

		app.clear();
		app.draw(plane);
		app.draw(cube.getMesh());

		app.display();

		lastFrameTime = currentFrameTime;
	}
}

void CollResDemo() {
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 20.0f));

	Mesh plane = CreatePlane(10.0f);
	plane.setPos(glm::vec3(0.0f));
	glm::vec3 nPlane = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	Gravity grav = Gravity::Gravity(aGravity);

	Shader shader = Shader("resources/shader/core.vert", "resources/shaders/core_green.frag");

	RigidBody cube = RigidBody();
	Mesh cMesh = Mesh::Mesh(Mesh::CUBE);
	cube.setMesh(cMesh);
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));

	cube.translate(glm::vec3(0.0f, 5.0f, 0.0f));
	cube.setVel(glm::vec3(0.0f, 0.0f, 0.0f));

	cube.setMass(2.0f);

	int demo = 1;
	if (demo == 1) {
		cube.setAngVel(glm::vec3(0.0f, 0.0f, 0.5f));
		cube.setCor(1.0f);
	}
	else if (demo == 2) {
		cube.setAngVel(glm::vec3(0.1f, 0.1f, 0.1f));
		cube.setCor(0.7f);
	}
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));

	cube.addForce(&grav);

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 500.0;
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	bool pause = false;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		if (!pause) {
			accumulator += frameDeltaTime;

			while (accumulator >= fixedDeltaTime) {
				cube.setAcc(cube.applyForces(cube.getPos(), cube.getVel(), (float)totalTime, deltaTime));

				cube.setVel(cube.getVel() + cube.getAcc() * fixedDeltaTime);
				cube.setAngVel(cube.getAngVel() + cube.getAngAcc() * fixedDeltaTime);

				cube.translate(cube.getVel() * fixedDeltaTime);

				glm::mat3 angVelSkew = glm::matrixCross3(cube.getAngVel());
				glm::mat3 rot = glm::mat3(cube.getRotate());
				rot += angVelSkew * rot * fixedDeltaTime;
				rot = glm::orthonormalize(rot);
				cube.setRotate(glm::mat4(rot));

				std::vector<Vertex> verts = cube.getMesh().getVertices();
				glm::vec3 vertAv = glm::vec3(0.0f);
				int colCount = 0;
				float minY = 0.0f;
				glm::mat4 m = cube.getMesh().getModel();
				for (int vCount = 0; vCount < (int)verts.size(); ++vCount) {
					glm::vec4 vWorld = m * glm::vec4(verts[vCount].getCoord(), 1.0f);
					vWorld = vWorld / vWorld.w;
					if (vWorld.y < 0.0f) {
						vertAv += glm::vec3(vWorld);
						colCount++;
						if (vWorld.y < minY) {
							minY = vWorld.y;
						}
					}
				}

				if (vertAv != glm::vec3(0.0f)) {
					vertAv = vertAv / colCount - cube.getPos();
					cube.translate(glm::vec3(0.0f, -minY, 0.0f));
					// jr = -(1+e)vr.n / m-1 +n.(I(r1*n))*r1
					glm::vec3 v1 = (cube.getVel() + glm::cross(cube.getAngVel(), glm::vec3(vertAv)));
					float numer = -(1 + cube.getCor()) * glm::dot(v1, nPlane);
					float denom = (1 / cube.getMass()) + glm::dot(nPlane, glm::cross(cube.getInvInertia() * glm::cross(glm::vec3(vertAv), nPlane), glm::vec3(vertAv)));
					float j = numer / denom;

					glm::vec3 vNew = cube.getVel() + nPlane * j / cube.getMass();
					glm::vec3 wNew = cube.getAngVel() + j * cube.getInvInertia() * glm::cross(glm::vec3(vertAv), nPlane);

					cube.setVel(vNew);
					cube.setAngVel(wNew);
				}

				if (totalTime < 2.0f) {
					cube.setAngAccl(cube.getAngAcc() - cube.getAngAcc() * (fixedDeltaTime) / (2.0f - totalTime));
				}

				accumulator -= fixedDeltaTime;
				totalTime += fixedDeltaTime;
			}
		}

		app.clear();
		app.draw(plane);
		app.draw(cube.getMesh());

		app.display();

		lastFrameTime = currentFrameTime;
	}
}

void FrictionDemo() {
	Application app = Application::Application();
	CreateApplication(app, glm::vec3(0.0f, 4.0f, 20.0f));

	Mesh plane = CreatePlane(10.0f);
	plane.setPos(glm::vec3(0.0f));
	glm::vec3 nPlane = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::vec3 aGravity = glm::vec3(0.0f, -9.8f, 0.0f);
	Gravity grav = Gravity::Gravity(aGravity);

	Shader shader = Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag");

	RigidBody cube = RigidBody();
	Mesh cMesh = Mesh::Mesh(Mesh::CUBE);
	cube.setMesh(cMesh);
	cube.getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_green.frag"));

	cube.translate(glm::vec3(0.0f, 6.0f, 0.0f));
	cube.setVel(glm::vec3(0.0f, -0.0f, 0.0f));

	cube.setMass(2.0f);

	float mu = 0.6f;

	int demo = 2;
	if (demo == 1) {
		cube.setAngVel(glm::vec3(0.0f, 0.0f, 0.5f));
		cube.setCor(1.0f);
	}
	else if (demo == 2) {
		cube.setAngVel(glm::vec3(0.0f, 01.0f, 0.51f));
		cube.setCor(0.6f);
	}
	else if (demo == 3) {
		cube.setPos(glm::vec3(-2.0f, 3.0f, 0.0f));
		cube.setVel(glm::vec3(6.0f, 0.0f, 0.0f));
		cube.setCor(0.6f);
	}
	cube.scale(glm::vec3(1.0f, 3.0f, 1.0f));

	cube.addForce(&grav);

	double timeSpeed = 1.0;
	double totalTime = 0.0;
	double fixedDeltaTime = timeSpeed * 1.0 / 500.0; //0.002
	double accumulator = 0.0;
	double startTime = glfwGetTime();
	double lastFrameTime = startTime;

	bool pause = false;

	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double currentFrameTime = glfwGetTime();
		double frameDeltaTime = (currentFrameTime - lastFrameTime) * timeSpeed;

		app.doMovement((GLfloat)frameDeltaTime / (GLfloat)timeSpeed);

		if (!pause) {
			accumulator += frameDeltaTime;

			while (accumulator >= fixedDeltaTime) {
				cube.setAcc(cube.applyForces(cube.getPos(), cube.getVel(), (float)totalTime, deltaTime));

				cube.setVel(cube.getVel() + cube.getAcc() * fixedDeltaTime);
				cube.setAngVel(cube.getAngVel() + cube.getAngAcc() * fixedDeltaTime);

				cube.translate(cube.getVel() * fixedDeltaTime);

				glm::mat3 angVelSkew = glm::matrixCross3(cube.getAngVel());
				glm::mat3 rot = glm::mat3(cube.getRotate());
				rot += angVelSkew * rot * fixedDeltaTime;
				rot = glm::orthonormalize(rot);
				cube.setRotate(glm::mat4(rot));

				std::vector<Vertex> verts = cube.getMesh().getVertices();
				glm::vec3 vertAv = glm::vec3(0.0f);
				int colCount = 0;
				float minY = 0.0f;
				glm::mat4 m = cube.getMesh().getModel();
				for (int vCount = 0; vCount < (int)verts.size(); ++vCount) {
					glm::vec4 vWorld = m * glm::vec4(verts[vCount].getCoord(), 1.0f);
					vWorld = vWorld / vWorld.w;
					if (vWorld.y < 0.0f) {
						vertAv += glm::vec3(vWorld);
						colCount++;
						if (vWorld.y < minY) {
							minY = vWorld.y;
						}
					}
				}

				if (vertAv != glm::vec3(0.0f)) {
					glm::vec3 angV = cube.getAngVel();


					vertAv = vertAv / colCount - cube.getPos();
					cube.translate(glm::vec3(0.0f, -minY, 0.0f));
					vertAv[1] -= minY;
					// jr = -(1+e)vr.n / m-1 +n.(I(r1*n))*r1
					glm::vec3 v1 = (cube.getVel() + glm::cross(cube.getAngVel(), glm::vec3(vertAv)));

					float numer = -(1 + cube.getCor()) * glm::dot(v1, nPlane);
					float denom = (1 / cube.getMass()) + glm::dot(nPlane,
						glm::cross(cube.getInvInertia() * glm::cross(glm::vec3(vertAv), nPlane), glm::vec3(vertAv)));
					float j = numer / denom;

					glm::vec3 vNew = nPlane * j / cube.getMass();
					glm::vec3 wNew = j * cube.getInvInertia() * glm::cross(glm::vec3(vertAv), nPlane);

					cube.setVel(cube.getVel() + vNew);
					cube.setAngVel(cube.getAngVel() + wNew);

					//v1 = (cube.getVel() + glm::cross(cube.getAngVel(), glm::vec3(vertAv)));
					glm::vec3 vTan = v1 - glm::dot(v1, nPlane) * nPlane;
					glm::vec3 jTan = (-mu) * abs(j) * (vTan / glm::length(vTan));
					//float maxFW = glm::length(vTan);
					float maxFW = glm::length(cube.getAngVel() / glm::length(cube.getInvInertia() * glm::cross(vertAv, vTan / glm::length(vTan))));
					if (glm::length(jTan) > maxFW) {
						jTan = jTan / glm::length(jTan);
						jTan *= maxFW;
					} 
					/*float jTanNum = -glm::dot(v1, nPlane);
					float jTanDen = (1.0f / cube.getMass()) + glm::dot(cube.getInvInertia() * glm::cross(glm::cross(vertAv, vTan), vertAv), vTan);

					float jTan2 = jTanNum / jTanDen;
					float jTanLim = j * mu;
					if (jTan2 > jTanLim) {
						jTan2 = jTanLim;
					}

					glm::vec3 jTanVec = jTan2 * vTan / glm::length(vTan);*/

					ApplyImpulse(cube, vertAv, jTan);

					/*float dot = abs(glm::dot(angV, nPlane));
					if (dot<0.05) {
						float lengTh = glm::length2(cube.getAngVel());
						float tanVal = atan(lengTh);
						tanVal /= M_PI_2;
						angV.y *= tanVal;
						cube.setAngVel(angV);
					}*/
				}

				if (totalTime < 2.0f) {
					cube.setAngAccl(cube.getAngAcc() - cube.getAngAcc() * (fixedDeltaTime) / (2.0f - totalTime));
				}

				accumulator -= fixedDeltaTime;
				totalTime += fixedDeltaTime;
			}
		}

		app.clear();
		app.draw(plane);
		app.draw(cube.getMesh());

		app.display();

		lastFrameTime = currentFrameTime;
	}
}

// main function
int main()
{
	int demo = 2;
	while (demo < 13) {
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
		case 4:
			UDemo();
			break;
		case 5:
			U2Demo();
			break;
		case 6:
			ClothDemo();
			break;
		case 7:
			FlagDemo();
			break;
		case 8:
			FirstRB();
			break;
		case 9:
			ImpDemo();
			break;
		case 10:
			CollDetDemo();
			break;
		case 11:
			CollResDemo();
			break;
		case 12:
			FrictionDemo();
			break;
		default:
			BoxDemo();
			break;
		}
		demo++;
	}

	return EXIT_SUCCESS;
}