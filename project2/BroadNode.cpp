#include "BroadNode.h"



BroadNode::BroadNode()
{
}


BroadNode::~BroadNode()
{
}

void BroadNode::addTemp(RigidBody* rb){
	if (!m_temps.empty()) {
		for (int i = 0; i < m_temps.size(); i++) {
			if (m_temps[i] == rb) {
				return;
			}
		}
	}
	m_temps.push_back(rb);
}