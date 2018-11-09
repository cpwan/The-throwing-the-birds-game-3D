#ifndef OBSTACLESUBSIM_H
#define OBSTACLESUBSIM_H

#include "SubSim.h"

/*
 * 
 */
class ObstacleSubSim : public SubSim {
public:
	ObstacleSubSim(CommonSim* parent);

	virtual void addObjects() override;

	virtual void resetMembers() override;
	virtual bool advance(float time, float dt) override;

	virtual void drawSimulationParameterMenu() override;

#pragma region SettersAndGetters

	void setCustom(double custom);

#pragma endregion SettersAndGetters

private:
	double m_custom;

	int m_ground;
};

#endif