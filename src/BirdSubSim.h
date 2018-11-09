#ifndef BIRDSUBSIM_H
#define BIRDSUBSIM_H

#include "SubSim.h"

/*
 * 
 */
class BirdSubSim : public SubSim {
public:
	BirdSubSim(CommonSim* parent);

	virtual void addObjects() override;

	virtual void resetMembers() override;
	virtual bool advance(float time, float dt) override;

	virtual void drawSimulationParameterMenu() override;

#pragma region SettersAndGetters

	void setCustom(double custom);

#pragma endregion SettersAndGetters

private:
	double m_custom;

	int m_leftFoot;
	int m_rightFoot;
	int m_body;
};

#endif