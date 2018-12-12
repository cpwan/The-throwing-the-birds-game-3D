#include "ObstacleSubSim.h"
#include "Gui.h"

ObstacleSubSim::ObstacleSubSim(CommonSim* parent)
:	SubSim(parent, "Obstacle"), 
	m_friction(0.4), 
	m_initial(80.0), 
	m_elast(0.5),
	m_gravity(981.0),
	m_iterativeResolution(false),
	m_collisionIterations(5),
	m_multiContactResolution(MULTI_CONTACT_SEQ),
	m_splitTimestep(true),
	m_minDtRatio(0.2),
	m_sleepMode(true),
	m_sleepThreshold(1.0),
	m_speculativeContacts(false)
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
	obstacleA.setLinearVelocity(Eigen::Vector3d(0, 0, 0));
	//obstacleA.setAngularVelocity(Eigen::Vector3d(2, 0, 8));
	//obstacleA.setRotation(	Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()) *
	//						Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitY()));

	RigidObject& obstacleB = getObject(m_obstacleB);
	obstacleB.setPosition(Eigen::Vector3d(0, 40, 30));
	obstacleB.setLinearVelocity(Eigen::Vector3d(0, 0, 0));
	//obstacleB.setRotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) *
	//					Eigen::AngleAxisd(0 / 4, Eigen::Vector3d::UnitX()) *
	//					Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));

	RigidObject& obstacleC = getObject(m_obstacleC);
	obstacleC.setPosition(Eigen::Vector3d(0, 50, 30));
	obstacleC.setLinearVelocity(Eigen::Vector3d(0, 0, 0));



	RigidObject& obstacleD = getObject(m_obstacleD);
	obstacleD.setPosition(Eigen::Vector3d(20, 5, 30));
	obstacleD.setLinearVelocity(Eigen::Vector3d(200, -500, 0));
	obstacleD.setAngularVelocity(Eigen::Vector3d(0, 0, 0));
	obstacleD.setRotation(Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitX()) *
						Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));


	RigidObject& obstacleE = getObject(m_obstacleE);
	obstacleE.setPosition(Eigen::Vector3d(2, 10, 30));
	//obstacleE.setLinearVelocity(Eigen::Vector3d(0, 200, 0));
	obstacleE.setRotation(	Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));

	RigidObject& obstacleF = getObject(m_obstacleF);
	obstacleF.setPosition(Eigen::Vector3d(-2, 10, 30));
	//obstacleF.setAngularVelocity(Eigen::Vector3d(20, 0, 8));
	//obstacleF.setLinearVelocity(Eigen::Vector3d(0, 200, 0));
	obstacleF.setRotation(	Eigen::AngleAxisd(0.0 / 4, Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(0.0 / 2, Eigen::Vector3d::UnitY()));
}

bool ObstacleSubSim::advance(float time, float dt) 
{
	BlockObstacle* collisionMarker = (BlockObstacle*)&getObject(m_marker);
	
	// Get obstacles to operate on
	std::vector<BlockObstacle*> obstacles;
	getCollideables(obstacles);

	// Apply external forces
	for (BlockObstacle* obstacle : obstacles)
	{
		if (obstacle->getType() == ObjType::DYNAMIC && obstacle->isActive())
		{
			// Apply external forces
			const Eigen::Vector3d gravity = Eigen::Vector3d(0, -m_gravity, 0);
			obstacle->setForce(gravity);

			const double drag = 0.001;
			obstacle->applyForceToCOM(-obstacle->getLinearVelocity() * drag);
			obstacle->applyTorque(-obstacle->getAngularVelocity() * drag);

			// Move obstacle one timestep forward
			obstacle->setLinearVelocity(obstacle->getLinearVelocity() + obstacle->getForce() * dt);
			obstacle->setAngularVelocity(obstacle->getAngularVelocity() + obstacle->getTorque() * dt);
		}
	}
	
	// Perform collision iterations
	int32_t iterations = collide(obstacles, dt);
	//std::cout << "Iterations: " << iterations << std::endl;
		
	if (m_sleepMode)
	{
		// Put blocks to sleep that didn't move for a long time
		for (BlockObstacle* obstacle : obstacles)
		{
			if (obstacle->isActive())
			{
				if (obstacle->isMoving(m_sleepThreshold))
				{
					obstacle->resetCounter();
				}
				else
				{
					// Put to sleep if inactive for too many frames
					obstacle->addCounter();
					if (obstacle->exceededCounter(60))
					{
						// Reset reciprocal movement
						obstacle->setLinearVelocity(Eigen::Vector3d::Zero());
						obstacle->setAngularVelocity(Eigen::Vector3d::Zero());

						obstacle->setActive(false);
						obstacle->resetCounter();
					}
				}
			}
			else
			{
				// Wake up if an external force has moved this obstacle
				if (obstacle->isMoving(m_sleepThreshold / 2))
				{
					obstacle->setActive(true);
				}
			}
		}
	}
	
	return(false);
}

void ObstacleSubSim::getCollideables(std::vector<BlockObstacle*>& obstacles)
{
	obstacles.clear();
	obstacles.push_back((BlockObstacle*)&getObject(m_ground));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleA));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleB));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleC));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleD));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleE));
	obstacles.push_back((BlockObstacle*)&getObject(m_obstacleF));
}

int32_t ObstacleSubSim::collide(const std::vector<BlockObstacle*>& obstacles, float dt)
{
	BlockObstacle* collisionMarker = (BlockObstacle*)&getObject(m_marker);
	const double thres = 0.0005;

	// Terminate once whole timestep has been executed
	double hitTime = dt;
	if (hitTime < SMALL_NUMBER)
	{
		return(0);
	}

	// Always progress to prevent infinite recursion
	double minHitTime = m_splitTimestep ? std::max(dt * m_minDtRatio, thres) : dt;
	for (BlockObstacle* obstacle : obstacles)
	{
		obstacle->setTime(dt);
	}
	
	// Get all possible contact points
	std::vector<MacroContact> contacts;
	for (BlockObstacle* obstacle : obstacles)
	{
		for (BlockObstacle* other : obstacles)
		{
			if (obstacle->isHitActive(other))
			{
				// Estimate how soon obstacles are gonna hit if active
				const double time = obstacle->hitSweepTime(other, dt, contacts);

				// Make sure hitTime doesn't go below minimal movement
				hitTime = std::max(std::min(time, hitTime), minHitTime);
			}
		}
	}

	// Perform actual move
	for (BlockObstacle* obstacle : obstacles)
	{
		if (obstacle->isActive())
		{
			if (m_speculativeContacts)
			{
				// Use obstacle collision time to prevent penetration, however secondary motion will get lost!
				const double time = std::min(hitTime, obstacle->getTime());
				obstacle->move(time);

				// Speculative contacts see
				// https://wildbunny.co.uk/blog/2011/03/25/speculative-contacts-an-continuous-collision-engine-approach-part-1/
			}
			else
			{
				obstacle->move(hitTime);
			}
		}
	}

	// Resolve penetrations
	for (const Contact& contact : contacts)
	{
		if (dt > SMALL_NUMBER)
		{
			// Make sure to resolve penetrations smoothly in case of wedging. 
			// Be more careful the smaller the timestep. TODO: Dicrease depending on number of connected contacts
			const double smoothing = (hitTime / dt) * 0.5;
			contact.ours->resolvePenetration(contact.theirs, smoothing);
		}
	}

	// Only regard earliest contacts
	std::vector<MacroContact> hits;
	for (const MacroContact& contact : contacts)
	{
		// Make sure simultaneous contacts are executed
		if (contact.time < hitTime + thres)
		{
			hits.push_back(contact);
		}
	}

	// Sort contacts by time for added stability
	std::sort(hits.begin(), hits.end(), [](const MacroContact& a, const MacroContact& b)->bool {return(a.time < b.time); });
	
	// Look for and merge duplicates
	std::vector<MacroContact> responses;
	for (MacroContact& hit : hits)
	{
		// Already spent duplicates get reset to null
		if(hit.ours)
		{
			MacroContact response;
			response.ours = hit.ours;
			response.theirs = hit.theirs;
			response.location = Eigen::Vector3d::Zero();
			response.normal = Eigen::Vector3d::Zero();

			// Compute average between all hits on the same objects, weight earlier hits stronger
			double weight = 0.0;
			for (MacroContact& other : hits)
			{
				const double earliness = (std::max(hitTime, thres) / std::max(response.time, thres));
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
				// Get final weighted average
				response.location /= weight;
				response.normal /= weight;

				// Surface normal is not normalized to dampen multihits on angled surfaces
				responses.push_back(response);
			}

		}
	}

	if (m_iterativeResolution)
	{
		// Create leave nodes
		std::vector<std::vector<NodeContact>> leaves;
		for (BlockObstacle* obstacle : obstacles)
		{
			// In this case static blocks are not considered
			if (obstacle->isActive())
			{
				std::vector<NodeContact> leaf;
				for (const MacroContact& response : responses)
				{
					// Create microcontacts fo all connected
					if (response.ours == obstacle || response.theirs == obstacle)
					{
						NodeContact contact = NodeContact(response, m_collisionIterations);

						// Flip if receiver
						if (response.theirs == obstacle)
						{
							contact.normal = -contact.normal;
							contact.theirs = contact.ours;
							contact.ours = obstacle;
						}

						leaf.push_back(contact);
					}
				}
				leaves.push_back(leaf);
			}
		}

		// Sort leaves by number of connections
		std::sort(leaves.begin(), leaves.end(), [](const auto& a, const auto& b)->bool {return(a.size() < b.size()); });

		if (!leaves.empty())
		{
			// Resolve contacts
			resolve(leaves);
		}
	}
	else
	{
		// Compute all contacts at once
		std::vector<MicroContact> leaves;
		for (const MacroContact& response : responses)
		{
			addNode(response, leaves);
		}
		resolve(leaves);
	}
	
	// Recursion
	return(collide(obstacles, dt - hitTime) + 1);
}


bool ObstacleSubSim::addNode(const Contact& contact, std::vector<MicroContact>& out)
{
	// Ignore diverging contacts
	const double speed = contact.computeSpeed();
	if (speed > SMALL_NUMBER)
	{
		// Impact response
		Eigen::Vector3d slide;
		const double speed = contact.computeSpeedAndSlide(slide);
		out.push_back(MicroContact(contact, (1.0 + m_elast)));

		// Friction is equivalent to an impulse response with elasticity < 1
		if (m_friction > SMALL_NUMBER)
		{
			const double norm = slide.norm();
			if (norm > SMALL_NUMBER)
			{
				// Apply friction against slide direction
				out.push_back(MicroContact(contact, m_friction));
				out.back().normal = slide / norm;
			}
		}

		return(true);
	}
	return(false);
}

void ObstacleSubSim::resolve(std::vector<std::vector<NodeContact>>& leaves)
{
	// Keep iterating until all contacts have been resolved a sufficient amount of times
	bool madeContact;
	do
	{
		madeContact = false;
		for (std::vector<NodeContact>& leaf : leaves)
		{
			std::vector<MicroContact> contacts;
			for (NodeContact& node : leaf)
			{
				// Can stop after a given amount of iterations
				if (node.iterations > 0)
				{
					if (addNode(node, contacts))
					{
						// Remember how many iterations have been executed on this node
						node.iterations--;
					}
				}
			}

			// Resolve if any contacts were found
			if (!contacts.empty())
			{
				resolve(contacts);
				madeContact = true;
			}
		}
	} while (madeContact);
}

void ObstacleSubSim::resolve(const std::vector<MicroContact>& contacts)
{
	if (m_multiContactResolution == MULTI_CONTACT_OFF)
	{
		for (const MicroContact& contact : contacts)
		{
			const double speed = contact.computeResponse();
			contact.ours->applyImpulseMulti(-speed, contact.theirs, contact.normal, contact.location);
		}
	}
	else
	{
		const int32_t n = contacts.size();
		if (n > 0)
		{
			// Compute multicontact impulses
			Eigen::VectorXd b = Eigen::VectorXd(n);
			Eigen::MatrixXd A = Eigen::MatrixXd(n, n);
			std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> angulars(n);
			for (int i = 0; i < n; i++)
			{
				const MicroContact& ci = contacts[i];
				auto& angular = angulars[i];

				b[i] = ci.computeResponse();
				for (int j = 0; j < n; j++)
				{
					const Contact& cj = contacts[j];
					Eigen::Vector3d ownImpulse, theirImpulse;
					if (j == i)
					{
						// Compute diagonal, cache angular for later
						ci.ours->computeImpulse(ci.normal, ci.location, angular.first, ownImpulse);
						ci.theirs->computeImpulse(-ci.normal, ci.location, angular.second, theirImpulse);
					}
					else
					{
						// Compute non-diagonal
						cj.ours->computeImpulse(ci.ours, ci.location, cj.normal, cj.location, ownImpulse);
						cj.theirs->computeImpulse(ci.theirs, ci.location, -cj.normal, cj.location, theirImpulse);
					}

					// Compute impulse magnitude
					A(i, j) = ci.normal.dot(ownImpulse - theirImpulse);
				}
			}

			// Compute the responses sequentially as a good first guess
			Eigen::VectorXd x = Eigen::VectorXd(n);
			for (int i = 0; i < n; i++)
			{
				x(i) = b[i] / A(i,i);
			}

			// Make sure A is psd
			if (Eigen::LLT<Eigen::MatrixXd>(A).info() != Eigen::NumericalIssue)
			{
				if (m_multiContactResolution == MULTI_CONTACT_LCP)
				{
					Eigen::VectorXd rx = x;

					// Compute impulse magnitudes
					const double lam = 1.0;
					for (int k = 0; k < 500; k++)
					{
						const Eigen::VectorXd r = A * x + b;
						for (int i = 0; i < n; i++)
						{
							x(i) = std::max(0.0, x(i) - lam * r(i) / A(i,i));
						}
					}

					x = x.cwiseMax(0.0);

					const double diff = (rx - x).norm();
					if (diff > SMALL_NUMBER)
					{
						{
							const Eigen::VectorXd w = A * rx + b;
							const double energy = w.dot(rx);
							std::cout << "---BEFORE---" << std::endl;
							std::cout << "Energy: " << energy << " | js: ";
							for (int i = 0; i < n; i++)
							{
								std::cout << rx(i) << " ";
							}
							std::cout << " | ws: ";
							for (int i = 0; i < n; i++)
							{
								std::cout << w(i) << " ";
							}
							std::cout << std::endl;
							{
								std::cout << "---AFTER---" << std::endl;
								const Eigen::VectorXd w = A * x + b;
								const double energy = w.dot(x);
								std::cout << "Energy: " << energy << " | js: ";
								for (int i = 0; i < n; i++)
								{
									std::cout << x(i) << " ";
								}
								std::cout << " | ws: ";
								for (int i = 0; i < n; i++)
								{
									std::cout << w(i) << " ";
								}
								std::cout << std::endl;
							}
						}
					}
				}
			}

			// Apply computed impulse magnitude
			for (int i = 0; i < n; i++)
			{
				const Contact& c = contacts[i];
				const auto& angular = angulars[i];

				// Apply to contact
				c.ours->applyImpulse(-x(i), c.normal, angular.first);
				c.theirs->applyImpulse(-x(i), -c.normal, angular.second);
			}
		}
	}
}

void ObstacleSubSim::drawSimulationParameterMenu() {
	SubSim::drawSimulationParameterMenu();

	ImGui::InputDouble("Friction coefficient", &m_friction, 0, 0);
	ImGui::InputDouble("Initial speed", &m_initial, 0, 0);
	ImGui::InputDouble("Elasticity", &m_elast, 0, 0);
	ImGui::InputDouble("Gravity", &m_gravity, 0, 0);

	ImGui::Checkbox("Contact iteration", &m_iterativeResolution);
	if (m_iterativeResolution)
	{
		ImGui::InputInt("Max contact iterations", &m_collisionIterations);
	}

	const char* combo[3] = {"Off", "Sequential", "Simultaneous (LCP)"};
	ImGui::Combo("Simultaneous contacts", &m_multiContactResolution, combo, sizeof(combo) / sizeof(char*));

	ImGui::Checkbox("Split timestep", &m_splitTimestep);
	if (m_splitTimestep)
	{
		ImGui::InputDouble("Minimum dt ratio", &m_minDtRatio, 0, 0);
	}

	ImGui::Checkbox("Sleep mode", &m_sleepMode);
	if (m_sleepMode)
	{
		ImGui::InputDouble("Sleep movement threshold", &m_sleepThreshold, 0, 0);
	}

	ImGui::Checkbox("Speculative contacts", &m_speculativeContacts);
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