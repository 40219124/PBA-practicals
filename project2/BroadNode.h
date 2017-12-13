#pragma once

#include <vector>

#include "RigidBody.h"

class BroadNode
{
public:
	BroadNode();
	~BroadNode();

	void addPerm(RigidBody* r) { m_perms.push_back(r); }
	void addTemp(RigidBody* r) { m_temps.push_back(r); }

	std::vector<RigidBody*> getPerms() { return m_perms; }
	std::vector<RigidBody*> getTemps() { return m_temps; }

	void clearTemps() { m_temps.empty(); }

private:
	std::vector<RigidBody*> m_perms;
	std::vector<RigidBody*> m_temps;
};

