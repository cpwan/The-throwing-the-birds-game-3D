#include "BirdSubSim.h"
#include "ObstacleSubSim.h"
#include "Gui.h"
#include <igl/edges.h>
#include <Eigen/SVD>
BirdSubSim::BirdSubSim(CommonSim* parent)
: SubSim(parent, "Bird")
{
	m_mass = 1.0;
	m_floor_stiffness = 100.0;
	m_gravity << 0, -9.81, 0;
	m_spring.stiffness = 10000.0;
	spring_scale = 5.0;
	m_spring.damping = 0.1;
	m_floor_friction = 0.05;
	m_angle = 0.6;
	m_initial_impluse = 20;
	ground = -3.0;
	groundNormal << 0, 1, -0.5;
	groundNormal.normalize();
	groundNormal = groundNormal.transpose();
}

void BirdSubSim::addObjects() {

	m_leftFoot = addObject("foot.off", Eigen::Vector3d(1.0, 0.9, 0.1), 1.0);
	m_rightFoot = addObject("foot.off", Eigen::Vector3d(1.0, 0.9, 0.1), 1.0);
	m_body = addObject("sphere.obj", Eigen::Vector3d(1.0, 0.1, 0.1), 2.0);
	
	body = &getObject(m_body);
	body->getMesh(m_Vorig_init, m_Forig_init);
	body->setMesh(m_Vorig_init, m_Forig_init);
	body->setScale(1.0);
}

void BirdSubSim::resetMembers() {

	leftFoot = &getObject(m_leftFoot);
	leftFoot->setPosition(Eigen::Vector3d(-0.75, 9, 1.7));
	leftFoot->setRotation(	Eigen::AngleAxisd(-M_PI / 3, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));

	rightFoot = &getObject(m_rightFoot);
	rightFoot->setPosition(Eigen::Vector3d(0.75, 9, 1.7));
	rightFoot->setRotation(	Eigen::AngleAxisd(-M_PI / 3, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));

	body = &getObject(m_body);
	body->reset();
	Eigen::Vector3d starting_pos;
	Eigen::Vector3d starting_velocity;

	starting_pos << 0, 10, 0;
	starting_velocity << 0, m_initial_impluse*sin(m_angle), m_initial_impluse*cos(m_angle);
	//body->setPosition(Eigen::Vector3d(0, 10, 0));
	body->setMass(m_mass);


	body->setMesh(m_Vorig_init.rowwise() + starting_pos.transpose(), m_Forig_init);
	body->getMesh(m_Vorig_rigid, m_Forig_rigid);
	body->getMesh(m_Vorig, m_Forig);
	m_velocities = std::vector<Eigen::Vector3d>(m_Vorig.rows(), starting_velocity);
	// update spring info.
	igl::edges(m_Forig, m_edges);
	m_lengths.resize(m_edges.size());
	for (int i = 0; i < m_edges.rows(); i++) {
		m_lengths[i] =
			(m_Vorig.row(m_edges(i, 0)) - m_Vorig.row(m_edges(i, 1)))
			.norm();
	}
}

bool BirdSubSim::advance(float time, float dt) {


	ObstacleSubSim* obstacleSim = (ObstacleSubSim*)getSubSim("Obstacle");
	std::vector<BlockObstacle*> collideables;
	obstacleSim->getCollideables(collideables);
	std::cout << collideables.size() << std::endl;

	Eigen::MatrixXd V, V_new,V_rigid;
	Eigen::MatrixXi F;
	body->getMesh(V, F);
	Eigen::Vector3d oldCOM = V.colwise().mean();

	V_new = V;
	V_rigid = m_Vorig_rigid;
	std::vector<Eigen::Vector3d> f(V.rows(), Eigen::Vector3d::Zero());

	////
	// TODO: update V_new (position) with f (force)
	// use m_edges and m_lengths to get info. about springs as below
	// e.g., int a = m_edges(i, 0); int b = m_edges(i, 1); Eigen::Vector3d v0 = V.row(a);
	// when each vertex is lower than 0 (i.e., floor), compute penalty force with m_floor_stiffness
	std::vector<Eigen::Vector3d> fint(V.rows(), Eigen::Vector3d::Zero());
	std::vector<Eigen::Vector3d> fdamp(V.rows(), Eigen::Vector3d::Zero());
	std::vector<Eigen::Vector3d> fext(V.rows(), Eigen::Vector3d::Zero());
	std::vector<Eigen::Vector3d> fRigidCore(V.rows(), Eigen::Vector3d::Zero());

	for (int i = 0; i < V.rows(); i++) {
		Eigen::Vector3d newPos = V.row(i);
		Eigen::Vector3d rigidPos = V_rigid.row(i);
		Eigen::Vector3d dir = rigidPos - newPos;
		double len = dir.norm();
		//cout << "\nlength" << i << "is" << len << endl;
		if (len!=0){
			dir.normalize();
			fRigidCore[i] = spring_scale *m_spring.stiffness * len * dir;
		}
		//
		//cout << "dir:\n" << dir << endl;
		//
		//cout << "force\n" << i << "is" << fRigidCore[i] << endl;
	}


	for (int i = 0; i < m_edges.rows(); i++) {
		int a = m_edges(i, 0);
		int b = m_edges(i, 1);

		Eigen::Vector3d dir = V.row(a) - V.row(b);
		double len = dir.norm();
		dir.normalize();
		fint[a] += m_spring.stiffness * (m_lengths[i] - len) * dir;
		fint[b] += m_spring.stiffness * (m_lengths[i] - len) * -dir;
	}

	for (int i = 0; i < V.rows(); i++) {
		fext[i] = body->getMass() * m_gravity;
		//if (V(i, 1) < ground) {
		//	fext[i].y() -= m_floor_stiffness * (V(i, 1));
		//	fext[i].x() -= m_floor_friction * (V(i, 0));
		//	fext[i].z() -= m_floor_friction * (V(i, 2));
		//}
		//else {
		//	fext[i] = body->getMass() * m_gravity;
		//}

		fdamp[i] = m_spring.damping*m_velocities[i];
	}

	Eigen::Vector3d aPtOnContactSurface(0, ground, 0);

	for (int i = 0; i < V.rows(); i++) {
		m_velocities[i] += body->getMassInv() * dt*(fint[i] + fext[i] - fdamp[i]+ fRigidCore[i]);

		V_new.row(i) = V.row(i) + dt * m_velocities[i].transpose();
		
		// replace <ground with contact detection, e.g. (normal|constant) dot (x,y,z,1)<0
		if (V_new.row(i).dot(groundNormal) < ground) {
			Eigen::Vector3d deltaV = dt * m_velocities[i].transpose();

			double numerator = (aPtOnContactSurface.transpose() - V.row(i)).dot(groundNormal);
			double denominator=	deltaV.dot(groundNormal);
			double t = numerator / denominator;
			Eigen::Vector3d r = deltaV * (1 - t);
			Eigen::Vector3d fr = r - (groundNormal.dot(r))*groundNormal;

			V_new.row(i) -=r.dot(groundNormal)*groundNormal + fr * m_floor_friction; //comment this line to let the bird swim horizontally
			m_velocities[i] = m_velocities[i]- m_velocities[i].dot(groundNormal)*groundNormal;

		}
	}

	body->setMesh(V_new, F);

	


	Eigen::Vector3d newCOM = V_new.colwise().mean();
	Eigen::Vector3d rigidCOM = V_rigid.colwise().mean();
	
	Eigen::MatrixXd q, p;
	Eigen::MatrixXd A,R;
	p = V_new.rowwise() - newCOM.transpose();
	q= V_rigid.rowwise() - rigidCOM.transpose();

	A=(q.transpose() * q).ldlt().solve(q.transpose() * p);
	//cout << A << endl;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
	//get the rotation matrix
	R = svd.matrixV()*svd.singularValues().asDiagonal() * svd.matrixV().transpose();

	//new v rigid
	m_Vorig_rigid =(q*R).rowwise() + newCOM.transpose();
	

	



	leftFoot = &getObject(m_leftFoot);
	leftFoot->setPosition(R*(leftFoot->getPosition() - rigidCOM) + newCOM);
	leftFoot->setRotation(R*leftFoot->getRotationMatrix());
	rightFoot = &getObject(m_rightFoot);
	rightFoot->setPosition(R*(rightFoot->getPosition() - rigidCOM) + newCOM);
	rightFoot->setRotation(R*rightFoot->getRotationMatrix());





	//cout << "after\n" << m_Vorig_rigid << endl;
	//cout <<"sth here:"<< (p * p_t).ldlt().solve(p * q_t) << endl;
	//for (int i = 0; i < V_new.rows(); i++) {

	//	cout <<i<<":"<< V_new.row(i) << endl;

	//}
	//cout << "magic!" << endl;
	//body->getMesh(V_new, F);
	//for (int i = 0; i < V_new.rows(); i++) {
	//
	//	cout << i << ":" << V_new.row(i) << endl;

	//}
	return(false);
}

void BirdSubSim::drawSimulationParameterMenu() {
	SubSim::drawSimulationParameterMenu();

	ImGui::InputDouble("Bird stiffness", & m_spring.stiffness, 0, 0);
	ImGui::InputDouble("Bird damping", &m_spring.damping, 0, 0);
	ImGui::InputDouble("Bird angle", &m_angle, 0, 0);
	ImGui::InputDouble("Bird init speed", &m_initial_impluse, 0, 0);
	
	ImGui::InputDouble("Bird rigid stiffness ratio", &spring_scale, 0, 0);
	ImGui::InputDouble("Floor stiffness", &m_floor_stiffness, 0, 0);
	ImGui::InputDouble("Floor friction", &m_floor_friction, 0, 0);
	
	
}

void BirdSubSim::setCustom(double custom) {
	m_custom = custom;
}

