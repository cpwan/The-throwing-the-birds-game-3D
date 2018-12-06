#ifndef OBSTACLESUBSIM_H
#define OBSTACLESUBSIM_H

#include "BlockObstacle.h"
#include "SubSim.h"


#define SMALL_NUMBER 0.001
/*
 * 
 */
class ObstacleSubSim : public SubSim {
public:
	ObstacleSubSim(CommonSim* parent);
	virtual ~ObstacleSubSim() {};

	virtual void addObjects() override;

	virtual void resetMembers() override;
	virtual bool advance(float time, float dt) override;

	// Collision iteration (counts number of iterations)
	int32_t collide(const std::vector<BlockObstacle*>& obstacles, float dt, int32_t iteration);

	virtual void drawSimulationParameterMenu() override;

#pragma region SettersAndGetters

	void setFriction(double friction);
	void setInitial(double initial);
	void setElast(double elast);
	void setGravity(double gravity);

#pragma endregion SettersAndGetters

private:
	double m_friction;
	double m_initial;
	double m_elast;
	double m_gravity;

	int m_marker;
	int m_ground;

	int m_obstacleA;
	int m_obstacleB;
	int m_obstacleC;

	int m_obstacleD;
	int m_obstacleE;
	int m_obstacleF;

};

#endif