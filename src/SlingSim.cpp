#include "SlingSim.h"
#include "Gui.h"

#include "CommonSim.h"

SlingSim::SlingSim(CommonSim* parent)
: SubSimulation(parent, "Sling") 
{
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
