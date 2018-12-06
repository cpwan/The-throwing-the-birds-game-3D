#include "CommonSim.h"
#include "SubSim.h"

CommonSim::CommonSim()
	: Simulation()
{
}

void CommonSim::init() {

	for (SubSim* subSim : m_subSims) {
		subSim->addObjects();
	}

	reset();
}

void CommonSim::resetMembers() {

	for (auto &o : m_objects) {
		o.reset();
	}

	for (SubSim* subSim : m_subSims) {
		subSim->resetMembers();
	}
}

void CommonSim::updateRenderGeometry() {
	
	for (size_t i = 0; i < m_objects.size(); i++) {
		RigidObject &o = m_objects[i];

		if (o.getID() < 0) {
			m_renderVs.emplace_back();
			m_renderFs.emplace_back();
		}

		m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
	}
}

bool CommonSim::advance() {

	for (SubSim* subSim : m_subSims) {
		subSim->advance(m_time, m_dt);
	}

	// advance time
	m_time += m_dt;
	m_step++;

	return false;
}

void CommonSim::renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) {
	
	for (size_t i = 0; i < m_objects.size(); i++) {
		RigidObject &o = m_objects[i];
		if (o.getID() < 0) {

			if (i > 0) {
				o.setID(viewer.append_mesh());
			} else {
				o.setID(0);
			}

			size_t meshIndex = viewer.mesh_index(o.getID());
			viewer.data_list[meshIndex].show_lines = false;
			viewer.data_list[meshIndex].set_face_based(true);
			viewer.data_list[meshIndex].point_size = 2.0f;
			viewer.data_list[meshIndex].clear();
		}

		size_t meshIndex = viewer.mesh_index(o.getID());
		viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
		viewer.data_list[meshIndex].compute_normals();

		Eigen::MatrixXd color;
		o.getColors(color);
		viewer.data_list[meshIndex].set_colors(color);
	}
}

void CommonSim::updateSimulationParameters() {
	for (SubSim* subSim : m_subSims) {
		subSim->updateSimulationParameters();
	}
}

void CommonSim::drawSimulationParameterMenu() {
	for (SubSim* subSim : m_subSims) {
		subSim->drawSimulationParameterMenu();
	}
}
