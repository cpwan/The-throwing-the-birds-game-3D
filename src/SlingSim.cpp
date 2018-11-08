#include "SlingSim.h"
#include "Gui.h"

void SlingSim::init() {

}

void SlingSim::resetMembers() {

}

bool SlingSim::advance() {

	return(false);
}


void SlingSim::updateSimulationParameters() {

}

void SlingSim::drawSimulationParameterMenu() {
	SubSimulation::drawSimulationParameterMenu();

	ImGui::InputFloat("custom", &m_custom, 0, 0);
}
