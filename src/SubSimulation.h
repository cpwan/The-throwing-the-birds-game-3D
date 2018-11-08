#ifndef SUBSIMULATION_H
#define SUBSIMULATION_H

#include <igl/edges.h>

class CommonSim;
class CommonGui;

using namespace std;

/*
 * Simulation of a bird, a sling and obstacles
 */
class SubSimulation {
public:
	SubSimulation(CommonSim* parent, const char* name) 
	: s_name(name), p_parent(parent) { init(); }

	virtual void init();
	virtual void resetMembers();
	virtual bool advance();

	virtual void updateSimulationParameters();
	virtual void drawSimulationParameterMenu();

private:
	std::string s_name;
	CommonSim* p_parent;
};

#endif