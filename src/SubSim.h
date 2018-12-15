#ifndef SUBSIM_H
#define SUBSIM_H

#include "CommonSim.h"

/*
 * 
 */
class SubSim {
public:
	SubSim(CommonSim* parent, const char* name);

	// Override to add objects using addObject(..)
	virtual void addObjects();

	// Override to reset object params
	virtual void resetMembers() = 0;

	// Override to advance one given step
	virtual bool advance(float time, float dt) = 0;

	// Override to update when Gui params have been changed
	virtual void updateSimulationParameters();

	// Override to draw to Gui, generates header by default.
	virtual void drawSimulationParameterMenu();

#pragma region SettersAndGetters

	bool hasName(const std::string& name) const;
	SubSim* getSubSim(const std::string& name) const;

#pragma endregion SettersAndGetters

protected:

	// Add object to main simulation
	int addObject(const RigidObject& Object);

	// Add object by path
	int addObject(const std::string& path, Eigen::Vector3d color = Eigen::Vector3d(1.0f), double scale = 1.0);

	// Get object by index
	RigidObject& getObject(int i) const;

private:
	std::string m_name;
	CommonSim* p_parent;
};

#endif