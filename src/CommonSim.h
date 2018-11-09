#ifndef COMMONSIM_H
#define COMMONSIM_H

#include "Simulation.h"
#include <deque>
using namespace std;

class SubSim;

/*
 * Common simulation that holds multiple sub-simulations.
 * Provides an interface for these sub-simulations to communicate.
 */
class CommonSim : public Simulation {
public:
	CommonSim();

	// Initialisation of objects
	virtual void init() override; 
	virtual void resetMembers() override;

	// Update vertex data with loaded objects
	virtual void updateRenderGeometry() override;

	// Advance simulation on all sub-simulations
	virtual bool advance() override;

	// Render cached vertex data
	virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override;

#pragma region SettersAndGetters

#pragma endregion SettersAndGetters

	// Create and add given sub-simulation
	template<typename T>
	void addSubSim() {
		m_subSims.push_back(new T(this));
	}

	// Forward Gui calls to sub-simulations
	void updateSimulationParameters();
	void drawSimulationParameterMenu();

private:

	std::vector<SubSim*> m_subSims;

	std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
	std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering
};

#endif