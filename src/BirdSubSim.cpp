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
	m_initial_impluse = 50;
	ground = -3.0;
	groundNormal << 0, 1, 0;
	groundNormal.normalize();
	groundNormal = groundNormal.transpose();
	block_impulse = 100.0;
	bird_impulse_ratio = 1.5;
	birdAngVelocity << 0, 0, 0;
	birdRotation=Eigen::Quaterniond::Identity();
	bird_impulse_angle_ratio = 0.05;
	initLeftFoot << -0.75, 9, 1.7;
	initRightFoot << 0.75, 9, 1.7;
	starting_pos << 0, 10, 0;
	m_mssEnabledCollision = true;

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

	birdAngVelocity << 0, 0, 0;
	birdRotation = Eigen::Quaterniond::Identity();
}

bool BirdSubSim::advance(float time, float dt) {


	ObstacleSubSim* obstacleSim = (ObstacleSubSim*)getSubSim("Obstacle");
	std::vector<BlockObstacle*> collideables;
	obstacleSim->getCollideables(collideables);
	//std::cout << collideables.size() << std::endl;
	


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

	// Compute spring forces
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

	// Compute external forces and damping
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

	// Symplectic euler (and legacy ground )
	Eigen::Vector3d aPtOnContactSurface(0, ground, 0);
	for (int i = 0; i < V.rows(); i++) {
		m_velocities[i] += body->getMassInv() * dt*(fint[i] + fext[i] - fdamp[i]+ fRigidCore[i]);

		V_new.row(i) = V.row(i) + dt * m_velocities[i].transpose();
	

		// replace <ground with contact detection, e.g. (normal|constant) dot (x,y,z,1)<0
		/*
		if (V_new.row(i).dot(groundNormal) < ground) {
			Eigen::Vector3d deltaV = dt * m_velocities[i].transpose();

			double numerator = (aPtOnContactSurface.transpose() - V.row(i)).dot(groundNormal);
			double denominator=	deltaV.dot(groundNormal);
			double t = numerator / denominator;
			Eigen::Vector3d r = deltaV * (1 - t);
			Eigen::Vector3d fr = r - (groundNormal.dot(r))*groundNormal;

			V_new.row(i) -=r.dot(groundNormal)*groundNormal + fr * m_floor_friction; //comment this line to let the bird swim horizontally
			m_velocities[i] = m_velocities[i]- m_velocities[i].dot(groundNormal)*groundNormal*bird_impulse_ratio;

			//Eigen::Vector3d radius = V.row(i);
			//Eigen::Vector3d angular = body->getInertiaInvWorld() *radius.cross(groundNormal);
			//Eigen::Vector3d impulse = angular.cross(radius) + groundNormal * body->getMassInv();

			//birdAngVelocity -= -bird_impulse_angle_ratio * denominator * block_impulse / groundNormal.dot(impulse) * angular;
		}
		*/
	}

	// Hit detection against obstacles
	HitResult hit;
	int hitResult;
	for (BlockObstacle* obstacle : collideables) {
		for (int i = 0; i < V_new.rows(); i++) {

			hitResult = obstacle->hitTestPoint(oldCOM, V_new.row(i).transpose()	 - oldCOM, hit);
			if (hitResult) { 
				
				aPtOnContactSurface = hit.location;
							
				if (m_mssEnabledCollision)
				{
					////////////////// MSS BASED SYSTEM ///////////////////
					// Works by fixing mass points to the surface and letting the MSS system do the legwork.
					// Applies forces according to springs to the obstacles the bird collides with.
					// All applied impulses happen on delta-values, not actual velocities or forces,
					// they do however get applied every tick resulting in a similar accumulative 
					// force over all.
					const Eigen::Vector3d otherV = obstacle->getPointVelocity(aPtOnContactSurface);
					const Eigen::Vector3d deltaV = m_velocities[i] - otherV;
					const double denominator = deltaV.dot(hit.normal);
					if (denominator < 0.0)
					{
						V_new.row(i) = aPtOnContactSurface;

						// Apply impulse in surface direction
						// Force reduction (magic number 10) to prevent the bird from just pushing obstacles
						const float force = fRigidCore[i].dot(hit.normal) * dt;
						obstacle->applyImpulseSingle(force / 10, -hit.normal, aPtOnContactSurface);

						// Compute "sliding" velocity along the surface, decrease by friction
						// TODO: Who defines friction, bird or obstacle?
						// TODO: Actual mass points don't rotate, deltaV therefore ignores angular velocity.
						const double friction = obstacleSim->getFriction();
						const Eigen::Vector3d tangential = deltaV - hit.normal * (deltaV.dot(hit.normal));
						m_velocities[i] = otherV + tangential * (1.0 - friction);

						// Apply impulse in sliding direction
						const double frictionForce = tangential.norm();
						if (frictionForce > SMALL_NUMBER)
						{
							const Eigen::Vector3d frictionNormal = tangential / frictionForce;
							obstacle->applyImpulseSingle(frictionForce * friction, frictionNormal, aPtOnContactSurface);

							// Make the bird rotate
							Eigen::Vector3d angular, impulse;
							obstacle->computeImpulse(frictionNormal, aPtOnContactSurface, angular, impulse);
							// Magic number 1000 to keep rigid/soft consistent
							birdAngVelocity -= bird_impulse_angle_ratio / 10 * frictionForce * friction / hit.normal.dot(impulse) * angular;
						}

					}
				}
				else
				{
					////////////////// ""RIGID"" BODY BASED SYSTEM ///////////////////
					// Changes velocities for every mass point like a rigid body system would.
					Eigen::Vector3d deltaV = dt * m_velocities[i].transpose();
					Eigen::Vector3d surfaceNormal = hit.normal;

					double numerator = (aPtOnContactSurface.transpose() - V_new.row(i)).dot(surfaceNormal);
					double denominator = deltaV.dot(surfaceNormal);
					double t = numerator / denominator;
					Eigen::Vector3d r = deltaV * (1 - t);
					Eigen::Vector3d fr = r - (surfaceNormal.dot(r))*surfaceNormal;

					V_new.row(i) -= r.dot(surfaceNormal)*surfaceNormal + fr * m_floor_friction; //comment this line to let the bird swim horizontally
					m_velocities[i] = m_velocities[i] - m_velocities[i].dot(surfaceNormal)*surfaceNormal*bird_impulse_ratio;

					obstacle->applyImpulseSingle(denominator*block_impulse, -surfaceNormal, aPtOnContactSurface);

					Eigen::Vector3d angular, impulse;
					obstacle->computeImpulse(surfaceNormal, aPtOnContactSurface, angular, impulse);


					birdAngVelocity -= -bird_impulse_angle_ratio * denominator * block_impulse / surfaceNormal.dot(impulse) * angular;

				}
			}

		}
	}
		
	body->setMesh(V_new, F);

	// Recompute rigid core transform
	Eigen::Vector3d newCOM = V_new.colwise().mean();
	Eigen::Vector3d rigidCOM = V_rigid.colwise().mean();
	
	Eigen::MatrixXd q, p;
	Eigen::Matrix3d A,R;
	p = V_new.rowwise() - newCOM.transpose();
	q= V_rigid.rowwise() - rigidCOM.transpose();

	A=(q.transpose() * q).ldlt().solve(q.transpose() * p);

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
	
	//get the rotation matrix
	R = svd.matrixV()*svd.singularValues().asDiagonal() * svd.matrixV().transpose();

	//new v rigid
	m_Vorig_rigid =(q*R).rowwise() + newCOM.transpose();
	

	// Compute feet transform and rotate the bird
	Eigen::Quaterniond q_RML(leftFoot->getRotationMatrix());
	Eigen::Quaterniond q_RMR(rightFoot->getRotationMatrix());
	
	const double angle = birdAngVelocity.norm();
	if (angle > 0.0f)
	{
		const Eigen::Vector3d axis = birdAngVelocity / angle;
		const Eigen::AngleAxisd rotation = Eigen::AngleAxisd(angle * dt, axis);
		birdRotation = (rotation * birdRotation).normalized();

		q_RML = rotation * q_RML;
		q_RMR = rotation * q_RMR;
	}
	Eigen::Quaterniond q_R(R);

	
	Eigen::Matrix3d birdRotMat = (q_R*birdRotation).normalized().toRotationMatrix();
	q_RML = (q_R*q_RML).normalized();
	q_RMR = (q_R*q_RMR).normalized();


	leftFoot = &getObject(m_leftFoot);
	leftFoot->setPosition(newCOM + birdRotMat * (initLeftFoot - starting_pos));
	leftFoot->setRotation(q_RML);

	rightFoot = &getObject(m_rightFoot);
	rightFoot->setPosition(newCOM + birdRotMat * (initRightFoot - starting_pos));
	rightFoot->setRotation(q_RMR);
	
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
	ImGui::InputDouble("Block_impulse", &block_impulse, 0, 0);

	ImGui::InputDouble("Bird_impulse_ratio", &bird_impulse_ratio, 0, 0);
	ImGui::InputDouble("Bird_impulse_angle_ratio", &bird_impulse_angle_ratio, 0, 0);

	
	ImGui::Checkbox("MSS enabled collision", &m_mssEnabledCollision);
}


void BirdSubSim::setCustom(double custom) {
	m_custom = custom;
}

