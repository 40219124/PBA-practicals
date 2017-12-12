#include "Collider.h"

#include <cmath>  
#include <vector>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include "glm/ext.hpp"

Collider::Collider() {
	m_pos = glm::vec3(0.0f);
}

Collider::~Collider() {}

void Collider::rotate(const float &angle, const glm::vec3 &vect) {
	m_axes = glm::rotate(glm::mat4(m_axes), angle, vect);
}

bool Collider::testCollision(Collider &b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen) {
	// Add together values of types
	int x = (int)this->getType() + (int)b.getType();
	// Switch based on types involved
	switch (x) {
	case 2:
		return this->findCollPointOBBOBB(b, out, normOut, halfPen);
		break;
	case 3:
		if (this->getType() == plane) {
			return this->findCollPointPlaneOBB(b, out, normOut, halfPen);
		}
		else {
			return b.findCollPointPlaneOBB(*this, out, normOut, halfPen);
		}
		break;
	}
	return x == 1;
}

void Collider::getClosestPtPointObb(glm::vec3 p, glm::vec3 &out) {
	// Vector form point to bound centre
	glm::vec3 vec = p - this->getPos();
	// Initialise out to centre
	out = this->getPos();
	// For each axis
	for (int i = 0; i < 3; ++i) {
		// Difference projected on an axis
		float len = glm::dot(vec, this->getAxes()[i]);
		// Check if out of radii limits and cap
		if (len > this->getRadii()[i]) {
			len = this->getRadii()[i];
		}
		else if (len < -this->getRadii()[i]) {
			len = (-this->getRadii()[i]);
		}
		// Add distance along axis to output
		out += len * this->getAxes()[i];
	}
}

glm::vec3 Collider::closestPointsToObb(Collider &col, glm::vec3 verts[8], std::vector<int> &ins) {
	// Distance of shortest 
	float dist = INFINITY;
	// Average collision variables
	int count = 0;
	glm::vec3 total = glm::vec3(0.0f);
	for (int i = 0; i < 8; i++) {
		glm::vec3 vert = verts[i];
		// Create test vertex
		glm::vec3 test = glm::vec3(0.0f);
		// Find closest point on this Obb to p
		col.getClosestPtPointObb(vert, test);
		// Find difference between this obb and p
		glm::vec3 vec = vert - test;
		// Get length of difference squared
		float dtest = glm::dot(vec, vec);
		// if multiple contact places
		float error = 0.0001;
		if (abs(dist - dtest) < error) {
			count++;
			total += test;
		}
		// If the length is shorter than other tested lengths keep it and point
		else if (dtest < dist - error) {
			dist = dtest;
			count = 1;
			total = test;
		}
		// If vertex inside OBB, record index
		if (dtest < error) {
			ins.push_back(i);
		}
	}
	return total / count;
}

glm::vec3 deepestPoint(const glm::vec3 colPos, const glm::vec3 verts[8], const std::vector<int> &is, const glm::vec3 &n, float &revPen) {
	float low = INFINITY;
	int count = 0;
	glm::vec3 total = glm::vec3(0.0f);
	for (int i = 0; i < is.size(); ++i) {
		float len = glm::dot((verts[is[i]] - colPos), n);
		// if multiple contact places
		float error = 0.0001;
		if (abs(low - len) < error) {
			count++;
			total += verts[is[i]];
		}
		// If the length is shorter than other tested lengths keep it and point
		else if (len < low - error) {
			low = len;
			count = 1;
			total = verts[is[i]];
		}
	}
	revPen = low;
	return total / count;
}

bool Collider::findCollPointOBBOBB(Collider &b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen) {
	// If there is no collision return
	if (!this->testOBBOBB(b)) {
		return false;
	}
	/*
	* Find all vertices
	*/
	// Put colliders into an array for loop stuff
	Collider* colls[2] = { this, &b };
	glm::vec3 vertsA[8];
	glm::vec3 vertsB[8];
	// Place for collider array loop stuff
	for (int flip = 0; flip < 2; ++flip) {
		int vi = 0;
		// Calculate what the vertices are
		for (int i = -1; i < 2; i += 2) {
			for (int j = -1; j < 2; j += 2) {
				for (int k = -1; k < 2; k += 2) {
					// Calculate vertex of Obb
					glm::vec3 p = glm::vec3(colls[flip]->getPos());
					p += colls[flip]->getAxes()[0] * i * colls[flip]->getRadii()[0];
					p += colls[flip]->getAxes()[1] * j * colls[flip]->getRadii()[1];
					p += colls[flip]->getAxes()[2] * k * colls[flip]->getRadii()[2];
					if (flip == 0) {
						vertsA[vi] = p;
					}
					else {
						vertsB[vi] = p;
					}
					vi++;
				}
			}
		}
	}
	/*
	* Get the points inside of the other obb, and the average of their locations
	*/
	// Index trackers of vertex locations
	std::vector<int> inA;
	std::vector<int> inB;
	int faceShape = 0;

	glm::vec3 avgA = closestPointsToObb(b, vertsA, inA);
	glm::vec3 avgB = closestPointsToObb(*this, vertsB, inB);

	/*
	* Find the axis along which each obb penetrated the other the least
	*/
	int indexA = -1;
	float lowA = INFINITY;
	if (inA.size() > 0) {
		// Find collision normal
		for (int i = 0; i < 3; ++i) {
			float len = glm::dot(avgA - colls[1]->getPos(), colls[1]->getAxes()[i]);
			float pen = colls[1]->getRadii()[i] - abs(len);
			if (pen < lowA) {
				lowA = pen;
				indexA = i;
			}
		}
	}

	int indexB = -1;
	float lowB = INFINITY;
	if (inB.size() > 0) {
		// Find collision normal
		for (int i = 0; i < 3; ++i) {
			float len = glm::dot(avgB - colls[0]->getPos(), colls[0]->getAxes()[i]);
			float pen = colls[0]->getRadii()[i] - abs(len);
			if (pen < lowB) {
				lowB = pen;
				indexB = i;
			}
		}
	}
	/*
	* After finding axis of penetration, find the obb which penetrated most
	*/
	glm::vec3 avgCP;
	int index;
	// if verts from this pen'd
	if (indexA > -1) {
		// if verts from b pen'd too
		if (indexB > -1) {
			// take the one that penetrated most
			if (lowA > lowB) {
				index = indexA;
				faceShape = 1;
			}
			else {
				index = indexB;
				faceShape = 0;
			}
			// average collision point is average of a and b averages
			avgCP = (avgA + avgB) / 2.0f;
		}
		// just a pen'd
		else {
			index = indexA;
			faceShape = 1;
			avgCP = avgA;
		}
		// no a pen
	}
	else {
		// just b pen'd
		if (indexB > -1) {
			index = indexB;
			faceShape = 0;
			avgCP = avgB;
		}
		// no pen (this should do edge stuff i guess. Or maybe that should come before here)
		else {
			return false;
		}
	}
	/*
	* Working out of final returned values
	*/
	// Average collision point in space of object hit in face
	glm::vec3 avgInObb = avgCP - colls[faceShape]->getPos();

	normOut = colls[faceShape]->getAxes()[index];
	if (glm::dot(normOut, avgCP - colls[faceShape]->getPos()) < 0.0f && glm::dot(normOut, colls[(faceShape + 1) % 2]->getPos() - colls[faceShape]->getPos()) < 0.0f) {
		normOut *= -1;
	}
	// Second pass test
	float revPen;
	if ((faceShape + 1) % 2 == 0) {
		out = deepestPoint(colls[1]->getPos(), vertsA, inA, normOut, revPen);
	}
	else {
		out = deepestPoint(colls[0]->getPos(), vertsB, inB, normOut, revPen);
	}
	halfPen = 0.0001 + (colls[faceShape]->getRadii()[index] - fminf(1.0f, abs(revPen))) / 2.0f;

	// Return whether there was collision
	return true;
}

bool Collider::testOBBOBB(Collider &b) {
	float r1, r2;
	glm::mat3 rot, absRot;
	float error = 0.00001f;
	// Difference between two centres
	glm::vec3 t = b.getPos() - this->getPos();
	// t in 1's space (coordinate frame)
	t = glm::vec3(glm::dot(t, this->getAxes()[0]), glm::dot(t, this->getAxes()[1]), glm::dot(t, this->getAxes()[2]));

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			// Projection of 2's axes onto 1's axes
			rot[i][j] = glm::dot(this->getAxes()[i], b.getAxes()[j]);
			// Compute common subexpressions
			absRot[i][j] = abs(rot[i][j]) + error;
		}
		// Test axes L = 1[0], L = 1[1], L = 1[2]
		r1 = this->getRadii()[i];
		r2 = b.getRadii()[0] * absRot[i][0] + b.getRadii()[1] * absRot[i][1] + b.getRadii()[2] * absRot[i][2];
		if (abs(t[i]) > (r1 + r2)) { return false; }
	}
	// Test axes L = 2[0], L = 2[1], L = 2[2]
	for (int i = 0; i < 3; ++i) {
		r1 = this->getRadii()[0] * absRot[0][i] + this->getRadii()[1] * absRot[1][i] + this->getRadii()[2] * absRot[2][i];
		r2 = b.getRadii()[i];
		if (abs(t[0] * rot[0][i] + t[1] * rot[1][i] + t[2] * rot[2][i]) > (r1 + r2)) { return false; }
	}
	// Test axis L = 1[0] x 2[0]
	r1 = this->getRadii()[1] * absRot[2][0] + this->getRadii()[2] * absRot[1][0];
	r2 = b.getRadii()[1] * absRot[0][2] + b.getRadii()[2] * absRot[0][1];
	if (abs(t[2] * rot[1][0] - t[1] * rot[2][0]) > (r1 + r2)) { return false; }
	// Test axis L = 1[0] x 2[1]
	r1 = this->getRadii()[1] * absRot[2][1] + this->getRadii()[2] * absRot[1][1];
	r2 = b.getRadii()[0] * absRot[0][2] + b.getRadii()[2] * absRot[0][0];
	if (abs(t[2] * rot[1][1] - t[1] * rot[2][1]) > (r1 + r2)) { return false; }
	// Test axis L = 1[0] x 2[2]
	r1 = this->getRadii()[1] * absRot[2][2] + this->getRadii()[2] * absRot[1][2];
	r2 = b.getRadii()[0] * absRot[0][1] + b.getRadii()[1] * absRot[0][0];
	if (abs(t[2] * rot[1][2] - t[1] * rot[2][2]) > (r1 + r2)) { return false; }
	// Test axis L = 1[1] x 2[0]
	r1 = this->getRadii()[0] * absRot[2][0] + this->getRadii()[2] * absRot[0][0];
	r2 = b.getRadii()[1] * absRot[1][2] + b.getRadii()[2] * absRot[1][1];
	if (abs(t[0] * rot[2][0] - t[2] * rot[0][0]) > (r1 + r2)) { return false; }
	// Test axis L = 1[1] x 2[1]
	r1 = this->getRadii()[0] * absRot[2][1] + this->getRadii()[2] * absRot[0][1];
	r2 = b.getRadii()[0] * absRot[1][2] + b.getRadii()[2] * absRot[1][0];
	if (abs(t[0] * rot[2][1] - t[2] * rot[0][1]) > (r1 + r2)) { return false; }
	// Test axis L = 1[1] x 2[2]
	r1 = this->getRadii()[0] * absRot[2][2] + this->getRadii()[2] * absRot[0][2];
	r2 = b.getRadii()[0] * absRot[1][1] + b.getRadii()[1] * absRot[1][0];
	if (abs(t[0] * rot[2][2] - t[2] * rot[0][2]) > (r1 + r2)) { return false; }
	// Test axis L = 1[2] x 2[0]
	r1 = this->getRadii()[0] * absRot[1][0] + this->getRadii()[1] * absRot[0][0];
	r2 = b.getRadii()[1] * absRot[2][2] + b.getRadii()[2] * absRot[2][1];
	if (abs(t[1] * rot[0][0] - t[0] * rot[1][0]) > (r1 + r2)) { return false; }
	// Test axis L = 1[2] x 2[1]
	r1 = this->getRadii()[0] * absRot[1][1] + this->getRadii()[1] * absRot[0][1];
	r2 = b.getRadii()[0] * absRot[2][2] + b.getRadii()[2] * absRot[2][0];
	if (abs(t[1] * rot[0][1] - t[0] * rot[1][1]) > (r1 + r2)) { return false; }
	// Test axis L = 1[2] x 2[2]
	r1 = this->getRadii()[0] * absRot[1][2] + this->getRadii()[1] * absRot[0][2];
	r2 = b.getRadii()[0] * absRot[2][1] + b.getRadii()[1] * absRot[2][0];
	if (abs(t[1] * rot[0][2] - t[0] * rot[1][2]) > (r1 + r2)) { return false; }
	// If all tests passed
	return true;
}

bool Collider::findCollPointPlaneOBB(Collider &b, glm::vec3 &out, glm::vec3 &normOut, float &halfPen) {
	normOut = this->getAxes()[0];
	glm::vec3 rVec = glm::vec3(b.getRadii()[0] * abs(glm::dot(normOut, b.getAxes()[0])),
		b.getRadii()[1] * abs(glm::dot(normOut, b.getAxes()[1])),
		b.getRadii()[2] * abs(glm::dot(normOut, b.getAxes()[2])));
	float rLen = rVec[0] + rVec[1] + rVec[2];
	glm::vec3 pToC = b.getPos() - this->getPos();
	float toLen = glm::dot(pToC, normOut);
	if (toLen > rLen) {
		return false;
	}
	halfPen = (rLen - toLen) / 2.0f;
	out = b.getPos();
	for (int i = 0; i < 3; ++i) {
		glm::vec3 vec = b.getRadii()[i] * b.getAxes()[i];
		if (glm::dot(vec, normOut) > 0) {
			vec *= -1.0f;
		}
		else if (abs(glm::dot(vec, normOut)) < 0.01) {
			vec *= 0.0f;
		}
		out += vec;
	}
	return true;
}
