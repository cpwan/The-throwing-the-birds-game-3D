
#ifndef COMMONSIMULATION_H
#define COMMONSIMULATION_H

#include <igl/edges.h>
#include "Simulation.h"

class SubSimulation;

using namespace std;

/*
 * Simulation of a bird, a sling and obstacles
 */
class CommonSim : public Simulation {
public:
	CommonSim() : Simulation() { init(); }

	virtual void init() override;
	virtual void resetMembers() override;
	virtual void updateRenderGeometry() override;
	virtual bool advance() override;
	virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override;

	std::vector<SubSimulation*> m_subSimulations; // Simulations to call

private:

	std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
	std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering
};

#endif