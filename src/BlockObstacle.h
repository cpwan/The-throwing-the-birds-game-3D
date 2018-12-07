#ifndef BLOCKOBSTACLE_H
#define BLOCKOBSTACLE_H

#include "RigidObject.h"
#include <functional>


#define SMALL_NUMBER 0.00001

typedef std::function<void(const Eigen::Vector3d&, const Eigen::Vector3d&)> VertexLamba;
typedef std::function<void(const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double len)> EdgeLamba;

class BlockObstacle;

struct HitResult
{
	HitResult();

	Eigen::Vector3d insider;	// Hitpoint inside the mesh
	Eigen::Vector3d location;	// Hitpoint on the surface
	Eigen::Vector3d normal;		// Surface normal
	double depth;				// Penetration depth
	double times;				// Ray time

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Contact
{
	Contact();
	double computeSpeedAndSlide(Eigen::Vector3d& Slide) const;
	double computeSpeed() const;

	BlockObstacle* ours;
	BlockObstacle* theirs;
	Eigen::Vector3d location;	// Hitpoint on the surface
	Eigen::Vector3d normal;		// Surface normal
};

struct MacroContact : Contact
{
	MacroContact();
	double time;				// Contact time
};

struct NodeContact : Contact
{
	NodeContact(const Contact& Contact, int32_t iterations);
	int32_t iterations;			// Contact iterations
};

struct MicroContact : Contact
{
	MicroContact(const Contact& Contact, double coeff);
	double computeResponse() const;
	double coeff;				// Contact response multiplier
};

struct Simplex
{
	Eigen::Vector3d mine;
	Eigen::Vector3d theirs;
	Eigen::Vector3d support;
};

struct Interval
{
	Interval(const Eigen::Vector3d& f);
	Interval(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	bool intersect(const Interval& other, int32_t i) const;
	bool intersectAny(const Interval& other) const;
	bool intersectAll(const Interval& other) const;
	void expand(const Eigen::Vector3d& f);

	Eigen::Vector3d a;
	Eigen::Vector3d b;
};

/*
 * 
 */
class BlockObstacle : public RigidObject {
public:
	BlockObstacle(const Eigen::Vector3d& extend, const char* name, const ObjType t = ObjType::DYNAMIC);
	
	Eigen::Vector3d getPointVelocity(const Eigen::Vector3d& point) const;

	// Vertex iterators
	void forEachVertexLocal(VertexLamba func) const;
	void forEachVertexGlobal(VertexLamba func) const;
	
	// Edge iterators
	void forEachEdgeLocal(EdgeLamba func) const;
	void forEachEdgeGlobal(EdgeLamba func) const;

	// Move this obstacles over a given timestep
	void move(double dt);

	// Cheaper collision test
	bool isInRange(const BlockObstacle* other) const;
	Interval projectOntoNormals(const Eigen::Matrix3d& normals) const;
	bool testCollision(const BlockObstacle* other) const;

	// Support functions for GJK (unused)
	Eigen::Vector3d support(const Eigen::Vector3d& dir) const;
	Simplex support(const Eigen::Vector3d& dir, const BlockObstacle& other) const;

	// Collision functions based on everything being boxes
	int32_t trace(const Eigen::Vector3d& point, const Eigen::Vector3d& vector, HitResult& close, HitResult& far) const;
	bool hitTestPoint(const Eigen::Vector3d& center, const Eigen::Vector3d& offset, HitResult& hit) const;
	bool hitTestEdge(const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double len, HitResult& hit) const;
	bool hitTest(const BlockObstacle* other, std::vector<HitResult>& hits) const;

	// Continuous collision time estimation over a given timestep, returns minimum hitTime
	double hitSweepTime(BlockObstacle* other, double dt, std::vector<MacroContact>& contacts);

	// Does a hittest and resolves penetrations by moving objects instantly
	void resolvePenetration(BlockObstacle* other, double factor);

	// Returns whether this block can interact with another obstacle
	bool isHitActive(BlockObstacle* other) const;

	// Measures whether this obstacle is currently moving
	bool isMoving(double threshold) const;

	// Collision handling
	void applyImpulseSingle(double relative, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint);
	void applyImpulseMulti(double relative, BlockObstacle* other, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint);

	void applyImpulse(double j, const Eigen::Vector3d& normal, const Eigen::Vector3d& angular);
	void computeImpulse(const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint, Eigen::Vector3d& angular, Eigen::Vector3d& impulse) const;
	void computeImpulse(BlockObstacle* other, const Eigen::Vector3d& otherHitpoint, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint, Eigen::Vector3d& impulse) const;
	
private:

};

#endif