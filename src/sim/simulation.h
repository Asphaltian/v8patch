#pragma once

#include <box3d/box3d.h>

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rbx/engine.h"
#include "sim/clumps.h"
#include "sim/stepping.h"
#include "sim/winding.h"

namespace v8patch::sim {

[[nodiscard]] double getTick() noexcept;

struct Explosion
{
	rbx::Vector3 position;
	float blastRadius = 0.0F;
	float blastPressure = 0.0F;
};

struct Assembly;

struct AssemblyPrimitive
{
	Assembly* assembly = nullptr;
	rbx::Primitive* primitive = nullptr;
	rbx::Body* body = nullptr;
	b3ShapeId shape = b3_nullShapeId;
	rbx::CoordinateFrame meInAssembly;
	rbx::CoordinateFrame hashedCoord;
	rbx::PV sentPv;
	rbx::Vector3 builtSize;
	float partMass = 0.0F;
	rbx::Geometry::Type builtType = rbx::Geometry::None;
	rbx::MotorJoint* motor = nullptr;
	std::int32_t parent = -1;
	std::int32_t stateIndex = 0;
	bool parentIsPrim0 = true;
	bool canCollide = false;
	bool live = false;
};

struct Assembly
{
	b3BodyId id = b3_nullBodyId;
	b3BodyType bodyType = b3_staticBody;
	std::vector<AssemblyPrimitive> primitives;
	std::uint64_t primitiveHash = 0;
	rbx::CoordinateFrame publishedCoord;
	b3Vec3 simLinear = b3Vec3_zero;
	b3Vec3 simRotational = b3Vec3_zero;
	bool anchored = false;
	bool hasMotor = false;
	bool jointed = false;
	bool clean = false;
	bool simOwned = false;
	bool unresolved = false;
};

struct Connector
{
	b3JointId id = b3_nullJointId;
	b3JointId mate = b3_nullJointId;
	b3BodyId bodyA = b3_nullBodyId;
	b3BodyId bodyB = b3_nullBodyId;
	rbx::Joint* joint = nullptr;
	rbx::Primitive* axle = nullptr;
	rbx::Joint::Type jointType = rbx::Joint::NoJoint;
	b3Vec3 baseInAxle = b3Vec3_zero;
	b3Vec3 rayInAxle = b3Vec3_zero;
	b3Vec3 markerInAxle = b3Vec3_zero;
	b3Vec3 markerInHole = b3Vec3_zero;
	float servoK = 0.0F;
	float goal = 0.0F;
	Winding winding;
	std::uint64_t pass = 0;
	bool broken = false;
};

class Simulation
{
public:
	bool open();
	void close();

	float step(rbx::World* world, float desiredInterval);
	void update(rbx::World* world);
	void invalidate() noexcept;

	bool doBreakJoints(rbx::World* world);
	void doBlast(const Explosion& explosion, const rbx::Array<rbx::Primitive*>& found);
	void computeFallen(rbx::Array<rbx::Primitive*>& fallen);

private:
	void findClumps(const rbx::Array<rbx::Primitive*>& primitives);
	void cleanAssemblies(const rbx::Array<rbx::Primitive*>& primitives);
	[[nodiscard]] bool isMotorChild(const rbx::Primitive* primitive) const;
	[[nodiscard]] bool resolved() const noexcept;
	void createAssembly(const rbx::Array<rbx::Primitive*>& primitives, std::size_t at);
	void buildAssembly(rbx::Primitive* assemblyPrimitive, bool anchored);
	void createGeometry(Assembly& assembly, AssemblyPrimitive& part);
	void setShape(AssemblyPrimitive& part);
	void setBodyType(Assembly& assembly);
	void setBranchMass(Assembly& assembly);

	[[nodiscard]] AssemblyPrimitive* getAssemblyPrimitive(const rbx::Primitive* primitive) noexcept;

	void stepUi();

	void updateBodies();
	void stepKernel(rbx::KernelData* data, bool throttling, bool sweeping);
	bool accumulateForces(Assembly& assembly, float dt, bool throttling, bool gate);

	void publishBodies(rbx::World* world, bool notify);
	void publishVelocity(Assembly& assembly);
	void reportTouches(rbx::World* world);

	static bool filterShapes(b3ShapeId first, b3ShapeId second, void* context);
	[[nodiscard]] static std::uint64_t pairKey(const void* first, const void* second) noexcept;

	void insertConnectors();
	void insertConnector(rbx::Joint* joint, rbx::Joint::Type jointType);
	void computeForce(Connector& connector, int uiStepId);
	void driveConnectors(int uiStepId);
	void findBreakingConnectors();
	void removeConnector(Connector& connector);

	b3WorldId world_ = b3_nullWorldId;
	b3Vec3 gravityStep_ = b3Vec3_zero;
	float spinRegain_ = 1.0F;
	rbx::KernelData* kernel_ = nullptr;

	std::vector<std::unique_ptr<Assembly>> assemblies_;
	std::unordered_map<rbx::Joint*, Connector> connectors_;

	Clumps clumps_;
	std::vector<AssemblyPrimitive*> primitiveToPart_;
	std::vector<Assembly*> forced_;
	std::vector<Assembly*> unresolved_;
	std::vector<b3BodyId> awakeIds_;
	std::unordered_map<std::uint64_t, Assembly*> byHash_;
	std::unordered_map<const rbx::Primitive*, std::int32_t> clumpSlot_;
	std::vector<std::size_t> dirtyClumps_;
	std::vector<rbx::Joint*> connectorJoints_;
	std::unordered_set<const rbx::Joint*> liveJoints_;
	std::unordered_set<const rbx::Primitive*> livePrimitives_;
	std::unordered_set<const rbx::Primitive*> wasStatic_;
	std::unordered_set<std::uint64_t> matedPairs_;
	std::vector<rbx::Joint*> breakingJoints_;
	std::unordered_set<const rbx::Primitive*> reportedFallen_;

	std::uint64_t primitiveSignature_ = 0;
	std::uint64_t anchors_ = 0;
	std::uint64_t connectorPass_ = 0;
	double entered_ = 0.0;
	Clock clock_;
	std::int32_t sinceScan_ = 0;
	std::int32_t primitiveCount_ = -1;
	std::int32_t jointCount_ = -1;
	bool connectorsPending_ = false;
};

} // namespace v8patch::sim
