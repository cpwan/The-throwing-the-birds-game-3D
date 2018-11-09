#include "SlingSubSim.h"
#include "Gui.h"

SlingSubSim::SlingSubSim(CommonSim* parent)
	: SubSim(parent, "Sling")
{
}

void SlingSubSim::addObjects() {

	m_sling = addObject("sling.off", Eigen::Vector3d(0.30, 0.17, 0.07), 1.0);
}

void SlingSubSim::resetMembers() {

	RigidObject& sling = getObject(m_sling);
	sling.setPosition(Eigen::Vector3d(0, 10, 0));

	sling.setRotation(	Eigen::AngleAxisd(-M_PI/16, Eigen::Vector3d::UnitX()) * 
						Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()));

}

bool SlingSubSim::advance(float time, float dt) {
	
	return(false);
}

void SlingSubSim::drawSimulationParameterMenu() {
	SubSim::drawSimulationParameterMenu();

	ImGui::InputDouble("Custom sling param", &m_custom, 0, 0);
}

void SlingSubSim::setCustom(double custom) {
	m_custom = custom;
}