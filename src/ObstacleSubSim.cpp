#include "ObstacleSubSim.h"
#include "Gui.h"

ObstacleSubSim::ObstacleSubSim(CommonSim* parent)
	: SubSim(parent, "Obstacle"), m_friction(0.4), m_initial(80.0), m_elast(0.5), m_gravity(981.0)
{

}

void ObstacleSubSim::addObjects() {

	m_ground = addObject(BlockObstacle(Eigen::Vector3d(300.0, 20.0, 300.0), "Ground", ObjType::STATIC));
	m_marker = addObject(BlockObstacle(Eigen::Vector3d(0.5, 0.5, 0.5), "Marker", ObjType::STATIC));

	m_obstacleA = addObject(BlockObstacle(Eigen::Vector3d(4.0, 4.0, 4.0), "A", ObjType::DYNAMIC));
	m_obstacleB = addObject(BlockObstacle(Eigen::Vector3d(3.0, 3.0, 3.0), "B", ObjType::DYNAMIC));
	m_obstacleC = addObject(BlockObstacle(Eigen::Vector3d(2.0, 2.0, 2.0), "C", ObjType::DYNAMIC));

	m_obstacleD = addObject(BlockObstacle(Eigen::Vector3d(4.0, 4.0, 4.0), "D", ObjType::DYNAMIC));
	m_obstacleE = addObject(BlockObstacle(Eigen::Vector3d(1.0, 2.0, 12.0), "E", ObjType::DYNAMIC));
	m_obstacleF = addObject(BlockObstacle(Eigen::Vector3d(1.0, 2.0, 12.0), "F", ObjType::DYNAMIC));
}

void ObstacleSubSim::resetMembers() {

	RigidObject& ground = getObject(m_ground);
	ground.setPosition(Eigen::Vector3d(0, -23, 0));
	ground.setColors(Eigen::Vector3d(0.2, 0.5, 0.2).transpose());

	RigidObject& marker = getObject(m_marker);
	marker.setColors(Eigen::Vector3d(1.0, 0.0, 0.0).transpose());


	RigidObject& obstacleA = getObject(m_obstacleA);
	obstacleA.setPosition(Eigen::Vector3d(0, 30, 30));
	obstacleA.setLinearVelocity(Eigen::Vector3d(0, 200, 0));
	//obstacleA.setAngularVelocity(Eigen::Vector3d(2, 0, 8));
	//obstacleA.setRotation(	Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()) *
	//						Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitY()));

	RigidObject& obstacleB = getObject(m_obstacleB);
	obstacleB.setPosition(Eigen::Vector3d(0, 40, 30));
	obstacleB.setLinearVelocity(Eigen::Vector3d(0, -200, 0));
	//obstacleB.setRotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) *
	//					Eigen::AngleAxisd(0 / 4, Eigen::Vector3d::UnitX()) *
	//					Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));

	RigidObject& obstacleC = getObject(m_obstacleC);
	obstacleC.setPosition(Eigen::Vector3d(0, 50, 30));
	obstacleC.setLinearVelocity(Eigen::Vector3d(0, 0, 0));



	RigidObject& obstacleD = getObject(m_obstacleD);
	obstacleD.setPosition(Eigen::Vector3d(20, -1, 30));
	obstacleD.setLinearVelocity(Eigen::Vector3d(0, 0, 0));

	RigidObject& obstacleE = getObject(m_obstacleE);
	obstacleE.setPosition(Eigen::Vector3d(2, 10, 30));
	//obstacleE.setLinearVelocity(Eigen::Vector3d(0, 200, 0));
	obstacleE.setRotation(	Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));

	RigidObject& obstacleF = getObject(m_obstacleF);
	obstacleF.setPosition(Eigen::Vector3d(-2, 10, 30));
	//obstacleF.setAngularVelocity(Eigen::Vector3d(20, 0, 8));
	obstacleF.setLinearVelocity(Eigen::Vector3d(0, 200, 0));
	obstacleF.setRotation(	Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));
}

bool ObstacleSubSim::advance(float time, float dt) {

	BlockObstacle* collisionMarker = (BlockObstacle*)&getObject(m_marker);
	
	std::vector<BlockObstacle*> obstacles;
	obstacles.push_back((BlockObstacle*)&getObject(m_ground));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleA));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleB));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleC));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleD));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleE));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleF));

	// Apply external forces
	for (BlockObstacle* obstacle : obstacles)
	{
		if (obstacle->getType() == ObjType::DYNAMIC && obstacle->isActive())
		{
			// Apply external forces
			const Eigen::Vector3d gravity = Eigen::Vector3d(0, -m_gravity, 0);
			obstacle->setForce(gravity);

			// Move obstacle one timestep forward
			obstacle->setLinearVelocity(obstacle->getLinearVelocity() + obstacle->getForce() * dt);
			obstacle->setAngularVelocity(obstacle->getAngularVelocity() + obstacle->getTorque() * dt);
		}
	}

	// Wake up everyone
	for (BlockObstacle* obstacle : obstacles)
	{
		if (!obstacle->isActive())
		{
			const double change = sqrt(
				obstacle->getLinearVelocity().squaredNorm() +
				obstacle->getAngularVelocity().squaredNorm());
			//std::cout << change << std::endl;

			//if (change > 2.0)
			{
				obstacle->setActive(true);
				obstacle->resetCounter();
			}
		}
	}

	// Perform collision iterations
	int32_t iterations = collide(obstacles, dt, 0);
	std::cout << "Iterations: " << iterations << std::endl;

	return(false);
}


int32_t ObstacleSubSim::collide(const std::vector<BlockObstacle*>& obstacles, float dt, int32_t iteration)
{
	BlockObstacle* collisionMarker = (BlockObstacle*)&getObject(m_marker);
	const double thres = 0.0005;

	// Terminate once whole timestep has been executed
	double hitTime = dt;
	if (hitTime < DBL_EPSILON)
	{
		return(0);
	}

	// Just apply full timestep on too many iterations to prevent stuttering
	double minHitTime = 0.0;
	if (iteration > 5)
	{
		std::cerr << "Terminated collision iteration early." << std::endl;
		minHitTime = dt;
	}
	
	// Get all possible contact points
	std::vector<Contact> contacts;
	for (BlockObstacle* obstacle : obstacles)
	{
		for (BlockObstacle* other : obstacles)
		{
			// Estimate how soon obstacles are gonna hit if active
			if (obstacle->isHitActive(other))
			{
				//std::cout << obstacle->getName() << " vs " << other->getName() << std::endl;
				// Hitsweep (using result from earlier iterations automatically minimises the hitTime)
				hitTime = obstacle->hitSweepTime(other, hitTime, minHitTime, contacts);
			}
		}
	}
	
	// Perform actual move
	for (BlockObstacle* obstacle : obstacles)
	{
		if (obstacle->isActive())
		{
			obstacle->move(hitTime);
		}
	}

	// Only regard earliest contacts
	std::vector<Contact> hits;
	for (const Contact& contact : contacts)
	{
		// Make sure simultaneous contacts are executed
		if (contact.time < hitTime + thres)
		{
			hits.push_back(contact);
		}
	}

	if (!hits.empty())
	{
		std::cout << "Hits generated: " << hits.size() << std::endl;
	}

	// Look for duplicates
	std::vector<Contact> responses;
	for (Contact& hit : hits)
	{
		// Already spent duplicates get reset to null
		if(hit.ours)
		{
			Contact response;
			response.ours = hit.ours;
			response.theirs = hit.theirs;
			response.location = Eigen::Vector3d::Zero();
			response.normal = Eigen::Vector3d::Zero();

			// Compute average between all hits on the same objects, weight earlier hits stronger
			double weight = 0.0;
			for (Contact& other : hits)
			{
				const double earliness = (hitTime / std::max(response.time, thres));
				const double hitWeight = earliness * earliness;
				if (other.ours == response.ours && other.theirs == response.theirs)
				{
					// Hit from the same detection cycle
					response.location += other.location * hitWeight;
					response.normal += other.normal * hitWeight;
					other.ours = nullptr;
					weight += hitWeight;
				}
				else if (other.ours == response.theirs && other.theirs == response.ours)
				{
					// Hit from other detection cycle, inverted normal
					response.location += other.location * hitWeight;
					response.normal -= other.normal * hitWeight;
					other.ours = nullptr;
					weight += hitWeight;
				}
			}

			if (weight > thres)
			{			
				// normalize
				response.location /= weight;
				response.normal /= weight;

				// Surface normal is not normalized to dampen multihits on angled surfaces
				responses.push_back(response);
			}

		}
	}

	// Resolve contacts
	for (const Contact& response : responses)
	{
		// Compute contact velocity
		const Eigen::Vector3d ownVelocity = response.ours->getPointVelocity(response.location);
		const Eigen::Vector3d theirVelocity = response.theirs->getPointVelocity(response.location);
		const Eigen::Vector3d relative = ownVelocity - theirVelocity;

		// Collision response
		response.ours->applyImpulseMulti(relative, response.theirs, 1.0 + m_elast, response.normal, response.location);
		collisionMarker->setPosition(response.location);

		// Friction response
		if (m_friction > 0.0)
		{
			// Get slide direction for friction
			const Eigen::Vector3d project = relative - response.normal * response.normal.dot(relative);
			const double slide = project.norm();
			if (slide > SMALL_NUMBER)
			{
				// Apply friction against slide direction
				response.ours->applyImpulseMulti(relative, response.theirs, m_friction, project / -slide, response.location);
			}
		}
	}

	// Resolve penetrations
	for (const Contact& response : responses)
	{
		response.ours->resolvePenetration(response.theirs);

		if (response.time < thres)
		{
			// Put to sleep if staying in place
			if (response.ours->exceededCounter(2) && !response.ours->isMoving(2.0))
			{
				response.ours->setActive(false);
			}

			response.ours->addCounter();
		}
	}

	// Notify how many iterations have been spent with hardly any progress
	if (hitTime < thres)
	{
		iteration++;
	}
	else
	{
		// Reset once things get moving again
		iteration = 0;
	}

	// Recursion
	return(collide(obstacles, dt - hitTime, iteration) + 1);
}

void ObstacleSubSim::drawSimulationParameterMenu() {
	SubSim::drawSimulationParameterMenu();

	ImGui::InputDouble("Friction coefficient", &m_friction, 0, 0);
	ImGui::InputDouble("Initial speed", &m_initial, 0, 0);
	ImGui::InputDouble("Elasticity", &m_elast, 0, 0);
	ImGui::InputDouble("Gravity", &m_gravity, 0, 0);
}

void ObstacleSubSim::setFriction(double friction) 
{
	m_friction = friction;
}

void ObstacleSubSim::setInitial(double initial)
{
	m_initial = initial;
}

void ObstacleSubSim::setElast(double elast)
{
	m_elast = elast;
}

void ObstacleSubSim::setGravity(double gravity)
{
	m_gravity = gravity;
}