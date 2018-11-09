#include "ObstacleSubSim.h"
#include "Gui.h"

ObstacleSubSim::ObstacleSubSim(CommonSim* parent)
	: SubSim(parent, "Obstacle")
{

}

void ObstacleSubSim::addObjects() {

	m_ground = addObject("cube.off", Eigen::Vector3d(0.1, 0.5, 0.1), 300.0);
}

void ObstacleSubSim::resetMembers() {

	RigidObject& ground = getObject(m_ground);
	ground.setPosition(Eigen::Vector3d(0, -303, 0));

}

bool ObstacleSubSim::advance(float time, float dt) {

	return(false);
}

void ObstacleSubSim::drawSimulationParameterMenu() {
	SubSim::drawSimulationParameterMenu();

	ImGui::InputDouble("Custom obstacle param", &m_custom, 0, 0);
}

void ObstacleSubSim::setCustom(double custom) {
	m_custom = custom;
}