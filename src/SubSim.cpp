#include "SubSim.h"
#include "Gui.h"

SubSim::SubSim(CommonSim* parent, const char* name)
: p_parent(parent), m_name(name)
{

}

void SubSim::addObjects() {

}

void SubSim::updateSimulationParameters() {
}

void SubSim::drawSimulationParameterMenu() {
	ImGui::Text(m_name.c_str());
}

int SubSim::addObject(const RigidObject& Object) {
	std::vector<RigidObject>& objects = p_parent->getObjects();
	objects.push_back(Object);
	return(objects.size()-1);
}

int SubSim::addObject(const std::string& path, Eigen::Vector3d color, double scale) {
	
	int i = addObject(RigidObject(path));
	RigidObject& o = getObject(i);

	o.setScale(scale);
	o.setType(ObjType::DYNAMIC);

	Eigen::MatrixXd col(1, 3);
	col << color.x(), color.y(), color.z();
	o.setColors(col);
	return(i);
}

RigidObject& SubSim::getObject(int i) const {
	std::vector<RigidObject>& objects = p_parent->getObjects();
	assert(0 <= i && i < objects.size() && "Object index out of bounds");
	return(objects[i]);
}

bool SubSim::hasName(const std::string& name) const
{
	return(m_name == name);
}

SubSim* SubSim::getSubSim(const std::string& name) const
{
	if (p_parent)
	{
		return(p_parent->getSubSim(name));
	}
	return(nullptr);
}