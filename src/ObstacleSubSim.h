#ifndef OBSTACLESUBSIM_H
#define OBSTACLESUBSIM_H

#include "BlockObstacle.h"
#include "SubSim.h"

#define MULTI_CONTACT_OFF 0
#define MULTI_CONTACT_SEQ 1
#define MULTI_CONTACT_LCP 2

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

	// Get objects active in collisions
	void getCollideables(std::vector<BlockObstacle*>& obstacles);

	// Collision iteration (counts number of iterations)
	int32_t collide(const std::vector<BlockObstacle*>& obstacles, float dt);

	// Resolve contacts
	void resolve(std::vector<std::vector<NodeContact>>& leaves);
	bool addNode(const Contact& contact, std::vector<MicroContact>& out);
	void resolve(const std::vector<MicroContact>& contacts);

	virtual void drawSimulationParameterMenu() override;

#pragma region SettersAndGetters

	void setFriction(double friction);
	void setInitial(double initial);
	void setElast(double elast);
	void setGravity(double gravity);

#pragma endregion SettersAndGetters

private:

	// General settings
	double m_friction;
	double m_initial;
	double m_elast;
	double m_gravity;

	// Collision settings
	bool m_iterativeResolution;
	int m_collisionIterations;
	int m_multiContactResolution;

	bool m_splitTimestep;
	double m_minDtRatio;

	bool m_sleepMode;
	double m_sleepThreshold;

	bool m_speculativeContacts;

	// Objects
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