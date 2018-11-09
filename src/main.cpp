#include <igl/writeOFF.h>

#include "CommonSim.h"
#include "Gui.h"

#include "BirdSubSim.h"
#include "ObstacleSubSim.h"
#include "SlingSubSim.h"


class CommonGui : public Gui {
public:
	float m_dt = 1e-3 * 3;

	CommonSim *p_CollisionSim = NULL;

	CommonGui() {

		// Initialise all sub-simulations
		p_CollisionSim = new CommonSim();
		p_CollisionSim->addSubSim<SlingSubSim>();
		p_CollisionSim->addSubSim<BirdSubSim>();
		p_CollisionSim->addSubSim<ObstacleSubSim>();
		p_CollisionSim->init();
		setSimulation(p_CollisionSim);

		// show vertex velocity instead of normal
		callback_clicked_vertex = [&](int clickedVertexIndex,
										int clickedObjectIndex,
										Eigen::Vector3d &pos,
										Eigen::Vector3d &dir) {
			RigidObject &o = p_CollisionSim->getObjects()[clickedObjectIndex];
			pos = o.getVertexPosition(clickedVertexIndex);
			dir = o.getVelocity(pos);
		};
		start();
	}

	virtual void updateSimulationParameters() override {
		p_CollisionSim->setTimestep(m_dt);

		if (p_CollisionSim) {
			p_CollisionSim->updateSimulationParameters();
		}
	}

	virtual void clearSimulation() override {
	}

	virtual void drawSimulationParameterMenu() override {
		ImGui::Text("Global");
		ImGui::InputFloat("dt", &m_dt, 0, 0);

		// Draw parameters of sub-simulations
		if (p_CollisionSim) {
			p_CollisionSim->drawSimulationParameterMenu();
		}
	}

};

int main(int argc, char *argv[]) {
    new CommonGui();

    return 0;
}