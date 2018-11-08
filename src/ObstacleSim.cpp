#include "ObstacleSim.h"
#include "Gui.h"

void ObstacleSim::init() {

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
