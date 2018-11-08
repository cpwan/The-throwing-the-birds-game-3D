
#ifndef SLINGSIMULATION_H
#define SLINGSIMULATION_H

#include <igl/edges.h>
#include "SubSimulation.h"

using namespace std;

/*
 * Simulation of a string with an attached mass.
 */
class SlingSim : public SubSimulation {
   public:
	   SlingSim(CommonSim* parent) : SubSimulation(parent, "Sling") { }

	   virtual void init() override;
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