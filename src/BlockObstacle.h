#ifndef BLOCKOBSTACLE_H
#define BLOCKOBSTACLE_H

#include "RigidObject.h"
#include <functional>

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
	double times;				// 
};

struct Contact
{
	Contact();
	BlockObstacle* ours;
	BlockObstacle* theirs;
	Eigen::Vector3d location;	// Hitpoint on the surface
	Eigen::Vector3d normal;		// Surface normal
	double time;				// Contact time
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
	double hitSweepTime(BlockObstacle* other, double dt, double min, std::vector<Contact>& contacts);

	// Does a hittest and resolves penetrations by moving objects instantly
	void resolvePenetration(BlockObstacle* other);

	// Returns whether this block can interact with another obstacle
	bool isHitActive(BlockObstacle* other) const;

	// Measures whether this obstacle is currently moving
	bool isMoving(double threshold) const;

	// Collision handling
	void BlockObstacle::applyImpulseSingle(const Eigen::Vector3d& relative, double coeff, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint);
	void BlockObstacle::applyImpulseMulti(const Eigen::Vector3d& relative, BlockObstacle* other, double coeff, const Eigen::Vector3d& normal, const Eigen::Vector3d& hitpoint);
	
private:

};

#endif