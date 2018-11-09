#include "BirdSubSim.h"
#include "Gui.h"

BirdSubSim::BirdSubSim(CommonSim* parent)
: SubSim(parent, "Bird")
{

}

void BirdSubSim::addObjects() {

	m_leftFoot = addObject("foot.off", Eigen::Vector3d(1.0, 0.9, 0.1), 1.0);
	m_rightFoot = addObject("foot.off", Eigen::Vector3d(1.0, 0.9, 0.1), 1.0);
	m_body = addObject("sphere.off", Eigen::Vector3d(1.0, 0.1, 0.1), 0.1);
}

void BirdSubSim::resetMembers() {

	RigidObject& leftFoot = getObject(m_leftFoot);
	leftFoot.setPosition(Eigen::Vector3d(-0.75, 9, 1.7));
	leftFoot.setRotation(	Eigen::AngleAxisd(-M_PI / 3, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));

	RigidObject& rightFoot = getObject(m_rightFoot);
	rightFoot.setPosition(Eigen::Vector3d(0.75, 9, 1.7));
	rightFoot.setRotation(	Eigen::AngleAxisd(-M_PI / 3, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));

	RigidObject& body = getObject(m_body);
	body.setPosition(Eigen::Vector3d(0, 10, 0));
}

bool BirdSubSim::advance(float time, float dt) {

	return(false);
}

void BirdSubSim::drawSimulationParameterMenu() {
	SubSim::drawSimulationParameterMenu();

	ImGui::InputDouble("Custom bird param", &m_custom, 0, 0);
}

void BirdSubSim::setCustom(double custom) {
	m_custom = custom;
}