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
	: s_name(name), p_parent(parent) { }

	virtual void resetMembers();
	virtual bool advance();

	virtual void updateSimulationParameters();
	virtual void drawSimulationParameterMenu();

protected:
	CommonSim* p_parent;

private:
	std::string s_name;
};

#endif