#include "BlockObstacle.h"

HitResult::HitResult()
: insider(Eigen::Vector3d::Zero()),
	location(Eigen::Vector3d::Zero()),
	normal(Eigen::Vector3d::Zero()),
	depth(0.0f),
	times(0.0f)
{
}

Contact::Contact()
:	ours(nullptr),
	theirs(nullptr),
	location(Eigen::Vector3d::Zero()),
	normal(Eigen::Vector3d::Zero())
{
}

MacroContact::MacroContact()
:	Contact(),
	time(0.0)
{
}

NodeContact::NodeContact(const Contact& Contact, int32_t iterations)
:	Contact(Contact),
	iterations(iterations)
{
}

MicroContact::MicroContact(const Contact& Contact, double coeff)
:	Contact(Contact),
	coeff(coeff)
{
}

double MicroContact::computeResponse() const
{
	return(computeSpeed() * coeff);
}

double Contact::computeSpeedAndSlide(Eigen::Vector3d& slide) const
{
	const Eigen::Vector3d ourVelocity = ours->getPointVelocity(location);
	const Eigen::Vector3d theirVelocity = theirs->getPointVelocity(location);
	const Eigen::Vector3d relative = theirVelocity - ourVelocity;
	const double speed = relative.dot(normal);
	slide = relative - normal * speed;
	return(speed);
}

double Contact::computeSpeed() const
{
	const Eigen::Vector3d ourVelocity = ours->getPointVelocity(location);
	const Eigen::Vector3d theirVelocity = theirs->getPointVelocity(location);
	return((theirVelocity - ourVelocity).dot(normal));
}

Interval::Interval(const Eigen::Vector3d& f)
	: a(f), b(f)
{
}

Interval::Interval(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
	: a(a), b(b)
{
}

bool Interval::intersect(const Interval& other, int32_t i) const
{
	return(b[i] >= other.a[i] && a[i] <= other.b[i]);
}

bool Interval::intersectAny(const Interval& other) const
{
	return(intersect(other, 0) || intersect(other, 1) || intersect(other, 2));
}

bool Interval::intersectAll(const Interval& other) const
{
	return(intersect(other, 0) && intersect(other, 1) && intersect(other, 2));
}

void Interval::expand(const Eigen::Vector3d& f)
{
	a = a.cwiseMin(f);
	b = b.cwiseMax(f);
}

BlockObstacle::BlockObstacle(const Eigen::Vector3d& extend, const char* name, const ObjType t)
	: RigidObject()
{
	setName(name);
	setExtend(extend);
	const double x = extend.x();
	const double y = extend.y();
	const double z = extend.z();

	//std::cout << " extendInit: " << x << ", " << y << ", " << z << std::endl;

	Eigen::MatrixXd Vertices = Eigen::MatrixXd(8, 3);
	Vertices <<		-x, -y, -z,		x, -y, -z,		-x, y, -z,		x, y, -z,
					-x, -y, z,		x, -y, z,		-x, y, z,		x, y, z;

	Eigen::MatrixXi Faces = Eigen::MatrixXi(12, 3);
	Faces <<	0, 2, 1,	2, 3, 1,	1, 3, 5,	3, 7, 5,
				2, 6, 3,	6, 7, 3,	5, 7, 4,	7, 6, 4,
				4, 6, 0,	6, 2, 0,	4, 0, 1,	4, 1, 5;

	setMesh(Vertices, Faces);

	Eigen::MatrixXd Colors = Eigen::MatrixXd(1, 3);
	Colors << 1.0f, 1.0f, 1.0f;
	setColors(Colors);

	setType(t);
	setMass(1.0);

	// inertia of cube
	const double tensor = getMass() / 12.0;
	setInertia((Eigen::Vector3d(y*y + z*z, x*x + z*z, x*x + y*y) * tensor).asDiagonal());
	reset();
}

Eigen::Vector3d BlockObstacle::getPointVelocity(const Eigen::Vector3d& point) const
{
	const Eigen::Vector3d ownRadius = point - getPosition();
	return(getLinearVelocity() + getAngularVelocity().cross(ownRadius));
}


void BlockObstacle::forEachVertexLocal(VertexLamba func) const
{
	const Eigen::Vector3d extend = getExtend();
	const float x = extend.x();
	const float y = extend.y();
	const float z = extend.z();
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(-x, -y, -z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(x, -y, -z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(-x, y, -z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(x, y, -z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(-x, -y, z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(x, -y, z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(-x, y, z));
	func(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(x, y, z));
}

void BlockObstacle::forEachVertexGlobal(VertexLamba func) const
{
	const Eigen::Vector3d source = getPosition();
	forEachVertexLocal([this, source, func](const Eigen::Vector3d& center, const Eigen::Vector3d& offset)
	{
		// Transform into worldspace
		func(source, getRotation() * offset * getScale());
	});
}

void BlockObstacle::forEachEdgeLocal(EdgeLamba func) const
{
	const Eigen::Vector3d extend = getExtend();
	const float x = extend.x();
	const float y = extend.y();
	const float z = extend.z();
	func(Eigen::Vector3d(x, y, z), Eigen::Vector3d(-1, 0, 0), 2 * x);
	func(Eigen::Vector3d(-x, y, z), Eigen::Vector3d(0, -1, 0), 2 * y);
	func(Eigen::Vector3d(-x, -y, z), Eigen::Vector3d(1, 0, 0), 2 * x);
	func(Eigen::Vector3d(x, -y, z), Eigen::Vector3d(0, 1, 0), 2 * y);
	
	func(Eigen::Vector3d(x, y, -z), Eigen::Vector3d(-1, 0, 0), 2 * x);
	func(Eigen::Vector3d(-x, y, -z), Eigen::Vector3d(0, -1, 0), 2 * y);
	func(Eigen::Vector3d(-x, -y, -z), Eigen::Vector3d(1, 0, 0), 2 * x);
	func(Eigen::Vector3d(x, -y, -z), Eigen::Vector3d(0, 1, 0), 2 * y);
	
	func(Eigen::Vector3d(x, y, z), Eigen::Vector3d(0, 0, -1), 2 * z);
	func(Eigen::Vector3d(-x, y, z), Eigen::Vector3d(0, 0, -1), 2 * z);
	func(Eigen::Vector3d(-x, -y, z), Eigen::Vector3d(0, 0, -1), 2 * z);
	func(Eigen::Vector3d(x, -y, z), Eigen::Vector3d(0, 0, -1), 2 * z);
}

void BlockObstacle::forEachEdgeGlobal(EdgeLamba func) const
{
	const Eigen::Vector3d source = getPosition();
	forEachEdgeLocal([this, source, func](const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double len)
	{
		// Transform into worldspace
		const Eigen::Vector3d& globalFrom = getRotation() * from * getScale();
		const Eigen::Vector3d& globalDir = getRotation() * dir;
		const double globalLen = len * getScale();
		func(source + globalFrom, globalDir, globalLen);
	});
}

void BlockObstacle::move(double dt)
{
	const Eigen::Vector3d prevPosition = getPosition();
	const Eigen::Quaterniond prevRotation = getRotation();

	const Eigen::Vector3d delta = getLinearVelocity() * dt;
	setPosition(prevPosition + delta);
	const double angle = getAngularVelocity().size();
	if (angle*angle > 0.0f)
	{
		const Eigen::Vector3d axis = getAngularVelocity() / angle;
		const Eigen::AngleAxisd rotation = Eigen::AngleAxisd(angle * dt, axis);
		const Eigen::Quaterniond quat = rotation * prevRotation;
		setRotation(quat.normalized());
	}
}

bool BlockObstacle::isInRange(const BlockObstacle* other) const
{
	const Eigen::Vector3d diff = getPosition() - other->getPosition();
	const double minDist = getExtend().norm() + other->getExtend().norm();
	return(diff.squaredNorm() < minDist * minDist);
}

Interval BlockObstacle::projectOntoNormals(const Eigen::Matrix3d& normals) const
{
	// Initialize with box center
	const Eigen::Vector3d source = getPosition();
	Interval interval = Interval(normals * source);
	forEachVertexGlobal([this, normals, &interval](const Eigen::Vector3d& center, const Eigen::Vector3d& offset)
	{
		// Expand interval with all vertices
		interval.expand(normals * (center + offset));
	});
	return(interval);
}

bool BlockObstacle::testCollision(const BlockObstacle* other) const
{
	// Project other vertices to own faces
	const Eigen::Matrix3d ownBase = getRotationMatrix().transpose();
	const Interval ownTheirs = other->projectOntoNormals(ownBase);
	const Interval ownOwn = projectOntoNormals(ownBase);

	if (ownTheirs.intersectAll(ownOwn))
	{
		// Project own vertices to other faces
		const Eigen::Matrix3d theirBase = other->getRotationMatrix().transpose();
		const Interval theirTheirs = other->projectOntoNormals(theirBase);
		const Interval theirOwn = projectOntoNormals(theirBase);

		return(theirTheirs.intersectAll(theirOwn));
	}
	return(false);
}


Eigen::Vector3d BlockObstacle::support(const Eigen::Vector3d& dir) const
{
	const Eigen::Vector3d extend = getExtend();
	const Eigen::Vector3d unit = (getRotation().inverse()*dir/getScale()).cwiseQuotient(extend);
	Eigen::Vector3d sqr = unit.cwiseQuotient(unit);
	if (isnan(unit.x())) sqr.x() = 0.0;
	if (isnan(unit.y())) sqr.y() = 0.0;
	if (isnan(unit.z())) sqr.z() = 0.0;
	return((getRotation()*sqr).cwiseProduct(extend));
}

Simplex BlockObstacle::support(const Eigen::Vector3d& dir, const BlockObstacle& other) const
{
	Simplex simplex;
	simplex.mine = support(dir);
	simplex.theirs = other.support(-dir);
	simplex.support = simplex.mine - simplex.theirs;
	return(simplex);
}

int32_t BlockObstacle::trace(const Eigen::Vector3d& point, const Eigen::Vector3d& vector, HitResult& close, HitResult& far) const
{
	// Transform point into unit-cube
	const Eigen::Vector3d extend = getExtend();
	const Eigen::Vector3d source = getPosition();
	const Eigen::Vector3d position = point - source;

	// Get box normals
	const Eigen::Matrix3d& normals = getRotationMatrix().transpose();

	// Compute offset multipliers for each face (can output nan)
	const Eigen::Vector3d divs = normals * vector;
	Eigen::VectorXd dists = Eigen::VectorXd(6);
	dists <<	(extend - normals * position).cwiseQuotient(divs),
				(-extend - normals * position).cwiseQuotient(divs);

	//std::cout << getName() << ": " << dists(0) << " " << dists(1) << " " << dists(2) << " " << dists(3) << " " << dists(4) << " " << dists(5) << " " << std::endl;

	// Get nearest valid intersection
	int32_t indexC = -1, indexF = -1;
	for (int i = 0; i < 6; i++)
	{
		// Check whether hit is within trace (deals with nans)
		const double dist = dists(i);
		if (0.0 <= dist && dist <= 1.0)
		{
			// Get smallest within block
			if (indexF < 0 || dist <= dists(indexF))
			{
				// Convert to unit cube to test whether hit is inside
				const Eigen::Vector3d location = position + vector * dist;
				const Eigen::Vector3d local = (normals*location).cwiseQuotient(extend);
				const Eigen::Vector3d absl = local.cwiseAbs();
				const double thres = 1.01;
				if (absl.x() <= thres && absl.y() <= thres && absl.z() <= thres)
				{
					// Check whether hit is closest
					if (indexC < 0 || dist <= dists(indexC))
					{
						// Old closest is now second closest
						indexF = indexC;
						indexC = i;
					}
					else
					{
						indexF = i;
					}
				}
			}
		}
	}

	if (indexC >= 0)
	{
		// Compute closer hitresult (indices above 3 are backfacing)
		close.times = dists(indexC);
		close.insider = point + vector;
		close.location = point + vector * close.times;
		close.normal = normals.row(indexC % 3) * (indexC > 2 ? -1.0 : 1.0);
		close.depth = close.normal.dot(close.location - point - vector);

		if (indexF >= 0)
		{
			// Compute further hitresult (indices above 3 are backfacing)
			far.times = dists(indexF);
			far.insider = point + vector;
			far.location = point + vector * far.times;
			far.normal = normals.row(indexF % 3) * (indexF > 2 ? -1.0 : 1.0);
			far.depth = far.normal.dot(far.location - point - vector);

			return(2);
		}
		return(1);
	}
	return(0);
}

bool BlockObstacle::hitTestPoint(const Eigen::Vector3d& center, const Eigen::Vector3d& offset, HitResult& hit) const
{
	HitResult temp;
	return(trace(center, offset, hit, temp) > 0);
}

bool BlockObstacle::hitTestEdge(const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double len, HitResult& hit) const
{
	HitResult close, far;

	const int32_t num = trace(from, dir * len, close, far);

	// The two hits and the hit location create a triangle with its height being the depth
	if (num > 1)
	{
		// No meaningful output for edge tracer
		hit.times = (close.times + far.times) / 2;

		// Check whether two opposite faces have been hit (dot == -1)
		if (far.normal.dot(close.normal) < -0.99)
		{
			// Hit on a point in the middle
			const Eigen::Vector3d extend = getExtend();
			const Eigen::Vector3d source = getPosition();
			hit.insider = (far.location + close.location) / 2;
			const Eigen::Vector3d diff = hit.insider - source;

			// We can assume the edge is perpendicular (otherwise it would have hit an edge normally beforehand)
			const Eigen::Matrix3d& normals = getRotationMatrix().transpose();

			// Get in local space
			const Eigen::Vector3d dists = normals * diff / getScale();
			const Eigen::Vector3d depths = extend - dists.cwiseAbs();
			const Eigen::Vector3d ratios = depths.cwiseQuotient(extend);
			
			// Get lowest depth
			int32_t index = -1;
			for (int i = 0; i < 3; i++)
			{
				// Get smallest within block
				const double dist = ratios(i);
				if (index < 0 || dist <= ratios(index))
				{
					index = i;
				}
			}

			// Flip normal if underside
			hit.depth = depths(index);
			hit.normal = normals.row(index);
			if (dists(index) < 0.0)
			{
				hit.normal = -hit.normal;
			}
			hit.location = hit.insider + hit.normal * hit.depth;
		}
		else
		{
			// Edge is the cross product between the intersected normals,
			// hit normal should be perpendicular to both edges
			hit.normal = far.normal.cross(close.normal).cross(dir);

			// Make sure normal points the right direction
			if (hit.normal.dot(close.normal) < 0.0 || hit.normal.dot(far.normal) < 0.0)
			{
				hit.normal = -hit.normal;
			}

			// Compute triangle
			const Eigen::Vector3d extend = close.location - far.location;
			const Eigen::Vector3d cathetus = close.normal * close.normal.dot(extend);
			hit.location = far.location + cathetus;
			hit.depth = hit.normal.dot(cathetus);
			hit.insider = hit.location - hit.normal * hit.depth;
		}
		return(true);
	}

	return(false);
}

bool BlockObstacle::hitTest(const BlockObstacle* other, std::vector<HitResult>& hits) const
{
	// Quick check for obstacles to far away from each other
	if (!isInRange(other))
	{
		return(false);
	}

	// Accumulate all hits on corners
	forEachVertexGlobal([&other, &hits](const Eigen::Vector3d& center, const Eigen::Vector3d& vertex)
	{
		HitResult hit;
		if (other->hitTestPoint(center, vertex, hit))
		{
			hits.push_back(hit);
		}
	});

	// Accumulate all hits on edges
	forEachEdgeGlobal([&other, &hits](const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double len)
	{
		HitResult hit;
		if (other->hitTestEdge(from, dir, len, hit))
		{
			hits.push_back(hit);
		}
	});

	return(!hits.empty());
}

double BlockObstacle::hitSweepTime(BlockObstacle* other, double dt, std::vector<MacroContact>& contacts)
{
	// Remember object states
	const Eigen::Vector3d prevOurPosition = getPosition();
	const Eigen::Quaterniond prevOurRotation = getRotation();
	const Eigen::Vector3d prevTheirPosition = other->getPosition();
	const Eigen::Quaterniond prevTheirRotation = other->getRotation();
	
	// Move both objects by given timestep
	move(dt);
	other->move(dt);
	double hitTime = dt;
	std::vector<HitResult> hits;
	if (hitTest(other, hits))
	{
		for (const HitResult& hit : hits)
		{
			// Create contact estimate
			MacroContact contact;
			contact.ours = this;
			contact.theirs = other;
			contact.location = hit.location;
			contact.normal = hit.normal;
			contact.time = dt;

			//std::cout << getName() << std::endl;
			//std::cout << hit.normal.x() << " " << hit.normal.y() << " " << hit.normal.z() << " " << hit.depth << std::endl;
			// Estimate time spent inside the other object using the velocity of the hitpoint
			const double normalSpeed = contact.computeSpeed();

			// Negative if diverging, still need to treat it as a contact in case of multicollision
			if (normalSpeed > SMALL_NUMBER)
			{
				// Compute time estimate, negative if already inside object from the start
				contact.time = std::max(dt - hit.depth / normalSpeed, 0.0);
			}

			// Update contact times
			minTime(contact.time);
			other->minTime(contact.time);
			hitTime = std::min(hitTime, contact.time);

			// Add to output
			contacts.push_back(contact);
		}
	}
	
	// Reset all bodies
	other->setPosition(prevTheirPosition);
	other->setRotation(prevTheirRotation);
	setPosition(prevOurPosition);
	setRotation(prevOurRotation);

	return(hitTime);
}

void BlockObstacle::resolvePenetration(BlockObstacle* other, double factor)
{
	std::vector<HitResult> hits;
	if (hitTest(other, hits))
	{
		// Apply highest penetration
		double maxDepth = 0.0;
		Eigen::Vector3d correction = Eigen::Vector3d::Zero();
		for (const HitResult& hit : hits)
		{
			if (hit.depth > maxDepth)
			{
				maxDepth = hit.depth;
				correction = hit.normal * maxDepth * factor;
			}
		}

		// Divide displacement if possible
		if (isActive() && other->isActive())
		{
			setPosition(getPosition() + correction * 0.5);
			other->setPosition(other->getPosition() - correction * 0.5);
		}
		else if (!other->isActive())
		{
			setPosition(getPosition() + correction);
		}
		else
		{
			other->setPosition(other->getPosition() - correction);
		}
	}
}

bool BlockObstacle::isHitActive(BlockObstacle* other) const
{
	// Collision needs to be checked if either of the two are active, also don't collide with self
	return(  other != this && (isActive() || other->isActive()));
}

bool BlockObstacle::isMoving(double threshold) const
{
	const double change = sqrt(
		getLinearVelocity().squaredNorm() +
		getAngularVelocity().squaredNorm());
	return(change > threshold);
}

double BlockObstacle::applyImpulseMulti(double relative, BlockObstacle* other, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint)
{
	if (relative > 0.0) return(0.0);

	// Compute response
	Eigen::Vector3d ownAngular, ownImpulse;
	computeImpulse(normal, hitpoint, ownAngular, ownImpulse);

	Eigen::Vector3d theirAngular, theirImpulse;
	other->computeImpulse(-normal, hitpoint, theirAngular, theirImpulse);

	// Compute impulse magnitude
	const double divisor = normal.dot(ownImpulse - theirImpulse);
	const double j = relative / divisor;
	
	// Apply to both
	applyImpulse(j, normal, ownAngular);
	other->applyImpulse(j, -normal, theirAngular);
	return(j);
}

double BlockObstacle::applyImpulseSingle(double relative, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint)
{
	if (relative > 0.0) return(0.0);

	// Compute response
	Eigen::Vector3d ownAngular, ownImpulse;
	computeImpulse(normal, hitpoint, ownAngular, ownImpulse);

	// Compute impulse magnitude
	const double ownInvMass = getMassInv();
	const double divisor = normal.dot(ownImpulse);
	const double j = relative / divisor;

	// Apply to own
	applyImpulse(j, normal, ownAngular);
	return(j);
}

void BlockObstacle::applyImpulse(double j, const Eigen::Vector3d& normal, const Eigen::Vector3d& angular)
{
	const double invMass = getMassInv();
	setAngularVelocity(getAngularVelocity() - j * angular);
	setLinearVelocity(getLinearVelocity() - j * invMass * normal);
}

void BlockObstacle::computeImpulse(const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint, Eigen::Vector3d& angular, Eigen::Vector3d& impulse) const
{
	const Eigen::Vector3d center = getPosition();
	const Eigen::Vector3d radius = hitpoint - center;
	const double invMass = getMassInv();

	angular = getInertiaInvWorld() * radius.cross(normal);
	impulse = angular.cross(radius) + normal * invMass;
}

void BlockObstacle::computeImpulse(const Eigen::Vector3d& center, const Eigen::Matrix3d& InertiaInvWorld, double MassInv, const Eigen::Vector3d& otherHitpoint, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint, Eigen::Vector3d& impulse) const
{
	const Eigen::Vector3d ownCenter = getPosition();
	const Eigen::Vector3d ownRadius = hitpoint - ownCenter;

	const Eigen::Vector3d theirRadius = otherHitpoint - center;

	const Eigen::Vector3d angular = InertiaInvWorld * ownRadius.cross(normal);
	impulse = angular.cross(theirRadius) + normal * MassInv;
}

void BlockObstacle::computeImpulse(BlockObstacle* other, const Eigen::Vector3d& otherHitpoint, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint, Eigen::Vector3d& impulse) const
{
	return(computeImpulse(other->getPosition(), other->getInertiaInvWorld(), other->getMassInv(), otherHitpoint, normal, hitpoint, impulse));
}