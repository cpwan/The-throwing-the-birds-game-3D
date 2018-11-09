#ifndef SLINGSUBSIM_H
#define SLINGSUBSIM_H

#include "SubSim.h"

/*
 * 
 */
class SlingSubSim : public SubSim {
public:
	SlingSubSim(CommonSim* parent);

	virtual void addObjects() override;

	virtual void resetMembers() override;
	virtual bool advance(float time, float dt) override;

	virtual void drawSimulationParameterMenu() override;

#pragma region SettersAndGetters

	void setCustom(double custom);

#pragma endregion SettersAndGetters

private:
	double m_custom;

	int m_sling;
	int m_blockA;
	int m_blockB;
};

#endif