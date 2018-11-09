#include "BirdSim.h"
#include "Gui.h"

BirdSim::BirdSim(CommonSim* parent)
: SubSimulation(parent, "Bird") 
{
}

void BirdSim::resetMembers() {

}

bool BirdSim::advance() {

	return(false);
}


void BirdSim::updateSimulationParameters() {

}

void BirdSim::drawSimulationParameterMenu() {
	SubSimulation::drawSimulationParameterMenu();

	ImGui::InputFloat("custom", &m_custom, 0, 0);
}
