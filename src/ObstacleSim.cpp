#include "ObstacleSim.h"
#include "Gui.h"

ObstacleSim::ObstacleSim(CommonSim* parent)
	: SubSimulation(parent, "Obstacle")
{
}

void ObstacleSim::resetMembers() {

}

bool ObstacleSim::advance() {

	return(false);
}


void ObstacleSim::updateSimulationParameters() {

}

void ObstacleSim::drawSimulationParameterMenu() {
	SubSimulation::drawSimulationParameterMenu();

	ImGui::InputFloat("custom", &m_custom, 0, 0);
}
