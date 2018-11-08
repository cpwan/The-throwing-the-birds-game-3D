#define EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "CommonSim.h"

#include "SlingSim.h"
#include "ObstacleSim.h"
#include "BirdSim.h"

/*
 * GUI for the spring simulation. This time we need additional paramters,
 * e.g. which integrator to use for the simulation and the force applied to the
 * canonball, and we also add some more visualizations (trajectories).
 */
class CommonGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    CommonSim *p_CommonSim = NULL;
	
	CommonGui() {

		p_CommonSim = new CommonSim();
        setSimulation(p_CommonSim);

		// Add subsimulations
		p_CommonSim->m_subSimulations.push_back(new SlingSim(p_CommonSim));
		p_CommonSim->m_subSimulations.push_back(new ObstacleSim(p_CommonSim));
		p_CommonSim->m_subSimulations.push_back(new BirdSim(p_CommonSim));

        // show vertex velocity instead of normal
        callback_clicked_vertex =
            [&](int clickedVertexIndex, int clickedObjectIndex,
                Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
                RigidObject &o = p_CommonSim->getObjects()[clickedObjectIndex];
                pos = o.getVertexPosition(clickedVertexIndex);
                dir = o.getVelocity(pos);
            };
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI
		p_CommonSim->setTimestep(m_dt);

		for (SubSimulation* subSim : p_CommonSim->m_subSimulations)
		{
			subSim->updateSimulationParameters();
		}
    }

    virtual void clearSimulation() override {
    }

    virtual void drawSimulationParameterMenu() override {

		ImGui::Text("Global");
		ImGui::InputFloat("dt", &m_dt, 0, 0);

		for (SubSimulation* subSim : p_CommonSim->m_subSimulations)
		{
			subSim->drawSimulationParameterMenu();
		}
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the spring simulation
    new CommonGui();

    return 0;
}