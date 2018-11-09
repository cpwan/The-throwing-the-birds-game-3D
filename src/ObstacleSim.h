
#ifndef OBSTACLESIMULATION_H
#define OBSTACLESIMULATION_H

#include <igl/edges.h>
#include "SubSimulation.h"

using namespace std;

/*
* Simulation of a string with an attached mass.
*/
class ObstacleSim : public SubSimulation {
public:
	ObstacleSim(CommonSim* parent);

	virtual void resetMembers() override;
	virtual bool advance() override;

	virtual void updateSimulationParameters() override;
	virtual void drawSimulationParameterMenu() override;

#pragma region SettersAndGetters

	void setCustom(float f) {
		m_custom = f;
	}

#pragma endregion SettersAndGetters

private:
	float m_custom;

};

#endif