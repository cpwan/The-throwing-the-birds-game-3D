#include "CommonSim.h"

void CommonSim::init() {

	m_dt = 1e-2;
	reset();
}

void CommonSim::resetMembers()
{

}

void CommonSim::updateRenderGeometry() {

	// Copy data from all objects that have an id, otherwise expand cache
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



	// advance m_time
	m_time += m_dt;
	m_step++;

	return false;
}

void CommonSim::renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) {

	for (size_t i = 0; i < m_objects.size(); i++) {

		RigidObject &o = m_objects[i];
		if (o.getID() < 0) {

			// Register new object
			o.setID(viewer.append_mesh());

			size_t meshIndex = viewer.mesh_index(o.getID());
			viewer.data_list[meshIndex].show_lines = false;
			viewer.data_list[meshIndex].set_face_based(true);
			viewer.data_list[meshIndex].point_size = 2.0f;
			viewer.data_list[meshIndex].clear();
		}

		// Render object
		size_t meshIndex = viewer.mesh_index(o.getID());

		viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
		viewer.data_list[meshIndex].compute_normals();

		Eigen::MatrixXd color;
		o.getColors(color);
		viewer.data_list[meshIndex].set_colors(color);
	}
}