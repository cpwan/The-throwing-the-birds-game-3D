#include "SubSimulation.h"
#include "Gui.h"


void SubSimulation::resetMembers()
{

}

bool SubSimulation::advance() {


	return false;
}

void SubSimulation::updateSimulationParameters() {

}

void SubSimulation::drawSimulationParameterMenu() {
	ImGui::Text(s_name.c_str());
}