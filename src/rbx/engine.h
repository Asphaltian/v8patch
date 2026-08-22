#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "target.h"

namespace v8patch::rbx {

struct Vector3
{
	float x{};
	float y{};
	float z{};

	friend bool operator==(const Vector3&, const Vector3&) = default;
};

struct Matrix3
{
	std::array<float, 9> row{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F};

	friend bool operator==(const Matrix3&, const Matrix3&) = default;
};

struct CoordinateFrame
{
	Matrix3 rotation;
	Vector3 translation;

	friend bool operator==(const CoordinateFrame&, const CoordinateFrame&) = default;
};

struct Quaternion
{
	float x{};
	float y{};
	float z{};
	float w{1.0F};
};

struct Velocity
{
	Vector3 linear;
	Vector3 rotational;
};

struct PV
{
	CoordinateFrame position;
	Velocity velocity;
};

static_assert(sizeof(Matrix3) == 0x24);
static_assert(sizeof(CoordinateFrame) == 0x30);
static_assert(sizeof(PV) == 0x48);

template <typename T>
struct Array
{
	T* items;
	std::int32_t count;
	std::int32_t capacity;

	[[nodiscard]] std::int32_t size() const noexcept { return items != nullptr ? count : 0; }
	[[nodiscard]] T* begin() const noexcept { return items; }
	[[nodiscard]] T* end() const noexcept { return items + size(); }
};

static_assert(sizeof(Array<void*>) == 0x0c);

template <typename Result, typename... Args>
Result invoke(const void* self, std::size_t slot, Args... args) noexcept
{
	using Method = Result(__thiscall*)(const void*, Args...);

	void* const* table = *static_cast<void* const* const*>(self);
	return reinterpret_cast<Method>(table[slot])(self, args...);
}

struct Geometry
{
	enum Type : int
	{
		Ball = 1,
		Block = 2,
		None = 3,
	};

	void* vftable;
	Vector3 gridSize;

	[[nodiscard]] Type type() const noexcept { return invoke<Type>(this, 2); }
	[[nodiscard]] float radius() const noexcept { return invoke<float>(this, 4); }
	[[nodiscard]] float gridVolume() const noexcept { return invoke<float>(this, 7); }
};

static_assert(offsetof(Geometry, gridSize) == 0x04);

struct Controller
{
	enum Input : int
	{
		NoInput = 0,
		LeftTrack = 1,
		RightTrack = 2,
		Constant = 12,
		Sin = 13,
	};

	[[nodiscard]] float value(Input input) const noexcept { return invoke<float>(this, 1, input); }
};

struct SurfaceData
{
	Controller::Input inputType;
	float paramA;
	float paramB;
};

static_assert(sizeof(SurfaceData) == 0x0c);

struct Body;

struct Cofm
{
	Body* body;
	std::uint8_t dirty;
	std::byte gap05[3];
	Vector3 cofmInBody;
	float mass;
	Matrix3 moment;
};

static_assert(offsetof(Cofm, cofmInBody) == 0x08);
static_assert(offsetof(Cofm, moment) == 0x18);

struct SimBody
{
	Body* body;
	std::uint8_t dirty;
	std::byte gap05[3];
	PV pv;
	Quaternion qOrientation;
	Vector3 angMomentum;
	Vector3 momentRecip;
	float massRecip;
	float constantForceY;
	Vector3 force;
	Vector3 torque;

	void clearAccumulators() noexcept
	{
		force = Vector3{0.0F, constantForceY, 0.0F};
		torque = Vector3{};
	}
};

static_assert(offsetof(SimBody, pv) == 0x08);
static_assert(offsetof(SimBody, qOrientation) == 0x50);
static_assert(offsetof(SimBody, momentRecip) == 0x6c);
static_assert(offsetof(SimBody, force) == 0x80);
static_assert(sizeof(SimBody) == 0x98);

struct Link
{
	void* vftable;
	Body* body;
	CoordinateFrame parentCoord;
	CoordinateFrame childCoord;
	CoordinateFrame childCoordInverse;
	mutable CoordinateFrame childInParent;
	mutable std::int32_t stateIndex;
};

static_assert(offsetof(Link, parentCoord) == 0x08);
static_assert(offsetof(Link, childCoordInverse) == 0x68);
static_assert(sizeof(Link) == 0xcc);

struct RevoluteLink : Link
{
	float jointAngle;
};

static_assert(offsetof(RevoluteLink, jointAngle) == 0xcc);

struct Body
{
	std::int32_t kernelIndex;
	Body* root;
	Body* parent;
	std::int32_t index;
	Array<Body*> children;
	Cofm* cofm;
	SimBody* simBody;
	std::uint8_t canThrottle;
	std::byte gap25[3];
	Link* link;
	CoordinateFrame meInParent;
	Matrix3 moment;
	float mass;
	std::int32_t stateIndex;
	PV pv;
};

static_assert(offsetof(Body, cofm) == 0x1c);
static_assert(offsetof(Body, link) == 0x28);
static_assert(offsetof(Body, meInParent) == 0x2c);
static_assert(offsetof(Body, mass) == 0x80);
static_assert(offsetof(Body, stateIndex) == 0x84);
static_assert(offsetof(Body, pv) == 0x88);
static_assert(sizeof(Body) == 0xd0);

struct Primitive;

struct Edge
{
	std::byte gap00[0x08];
	std::int32_t edgeState;
	Primitive* prim0;
	Primitive* prim1;
	Edge* next0;
	Edge* next1;
	std::uint8_t inEdgeList;
	std::byte gap1d[3];

	enum Type : int
	{
		JointEdge = 0,
		ContactEdge = 1,
	};

	[[nodiscard]] Type edgeType() const noexcept { return invoke<Type>(this, 3); }

	[[nodiscard]] Edge* next(const Primitive* from) const noexcept { return from == prim0 ? next0 : next1; }
	[[nodiscard]] Primitive* other(const Primitive* from) const noexcept { return from == prim0 ? prim1 : prim0; }
};

static_assert(offsetof(Edge, prim0) == 0x0c);
static_assert(sizeof(Edge) == 0x20);

struct EdgeList
{
	Edge* first;
	std::int32_t num;
};

struct Face
{
	std::array<Vector3, 4> corner;

	[[nodiscard]] float width() const noexcept;
	[[nodiscard]] float height() const noexcept;
};

static_assert(sizeof(Face) == 0x30);

struct Joint : Edge
{
	enum Type : int
	{
		NoJoint = 0,
		Rotate = 1,
		RotateP = 2,
		RotateV = 3,
		Glue = 4,
		Anchor = 5,
		Weld = 6,
		Snap = 7,
		Motor = 8,
		Free = 9,
	};

	void* jointOwner;
	std::uint8_t active;
	std::byte gap25[3];
	CoordinateFrame jointCoord0;
	CoordinateFrame jointCoord1;

	[[nodiscard]] Type jointType() const noexcept { return invoke<Type>(this, 5); }

	[[nodiscard]] const CoordinateFrame& jointCoord(int index) const noexcept
	{
		return index == 0 ? jointCoord0 : jointCoord1;
	}
};

static_assert(offsetof(Joint, jointCoord0) == 0x28);
static_assert(sizeof(Joint) == 0x88);

struct MultiJoint : Joint
{
	std::int32_t numConnector;
	std::array<void*, 8> point;
	std::array<void*, 4> connector;
	std::int32_t numBreakingConnectors;
};

static_assert(offsetof(MultiJoint, connector) == 0xac);
static_assert(sizeof(MultiJoint) == 0xc0);

struct GlueJoint : MultiJoint
{
	Face overlapInP0;
	Face overlapInP1;
};

static_assert(offsetof(GlueJoint, overlapInP0) == 0xc0);

struct MotorJoint : Joint
{
	RevoluteLink* link;
	float currentAngle;
	float maxVelocity;
	float desiredAngle;
};

static_assert(offsetof(MotorJoint, currentAngle) == 0x8c);
static_assert(sizeof(MotorJoint) == 0x98);

struct IMoving
{
	void* vftable;
	void* iMovingManager;
	std::int32_t stepsToSleep;

	[[nodiscard]] bool reportTouches() const noexcept { return invoke<bool>(this, 1); }
};

static_assert(sizeof(IMoving) == 0x0c);

enum class NormalId : std::int32_t
{
	X = 0,
	Y = 1,
	Z = 2,
	XNeg = 3,
	YNeg = 4,
	ZNeg = 5,
	Undefined = 6,
};

enum class SurfaceType : std::int32_t
{
	Smooth = 0,
	Glue = 1,
	Weld = 2,
	Studs = 3,
	Inlet = 4,
	Universal = 5,
	Hinge = 6,
	Motor = 7,
	SteppingMotor = 8,
};

struct World;

struct Primitive
{
	std::byte gap00[0x18];
	EdgeList contacts;
	EdgeList joints;
	std::int32_t worldIndex;
	World* world;
	void* clump;
	std::byte gap34[0x40];
	Geometry* geometry;
	Body* body;
	IMoving* myOwner;
	void* anchorObject;
	std::uint8_t dragging;
	std::uint8_t anchored;
	std::uint8_t canCollide;
	std::uint8_t canSleep;
	float friction;
	float elasticity;
	std::array<SurfaceType, 6> surfaceType;
	std::array<SurfaceData*, 6> surfaceData;
	Controller* controller;

	[[nodiscard]] Edge* firstJoint() const noexcept { return joints.first; }
	[[nodiscard]] float radius() const noexcept { return invoke<float>(this, 3); }
};

static_assert(offsetof(Primitive, contacts) == 0x18);
static_assert(offsetof(Primitive, worldIndex) == 0x28);
static_assert(offsetof(Primitive, geometry) == 0x74);
static_assert(offsetof(Primitive, dragging) == 0x84);
static_assert(offsetof(Primitive, friction) == 0x88);
static_assert(offsetof(Primitive, surfaceData) == 0xa8);
static_assert(offsetof(Primitive, controller) == 0xc0);

struct IStage
{
	enum StageType : int
	{
		JointStage = 0,
		ClumpStage = 1,
		TreeStage = 2,
		AssemblyStage = 3,
		CollisionStage = 4,
		SleepStage = 5,
		SeparateStage = 6,
		SimJobStage = 7,
		KernelStage = 8,
	};

	void* vftable;
	IStage* upstream;
	IStage* downstream;

	[[nodiscard]] StageType stageType() const noexcept { return invoke<StageType>(this, 1); }
};

static_assert(sizeof(IStage) == 0x0c);

struct Connector
{
	void* vftable;
	std::int32_t kernelIndex;

	void computeForce(float dt, bool throttling) noexcept { invoke<void>(this, 1, dt, throttling); }
};

struct KernelData
{
	Array<Body*> bodies;
	Array<void*> points;
	Array<Connector*> connectors;
	Array<Connector*> connectors2ndPass;
};

static_assert(offsetof(KernelData, connectors2ndPass) == 0x24);

struct Kernel
{
	IStage stage;
	std::uint8_t inStepCode;
	std::byte gap0d[3];
	KernelData* kernelData;
};

static_assert(offsetof(Kernel, kernelData) == 0x10);

struct World
{
	std::byte gap00[0x30];
	void* contactManager;
	IStage* jointStage;
	Array<Primitive*> touch;
	Array<Primitive*> touchOther;
	std::uint8_t canThrottle;
	std::uint8_t inStepCode;
	std::uint8_t inJointNotification;
	std::byte gap53;
	std::int32_t worldStepId;
	Array<Primitive*> primitives;
	std::byte gap64[0x0c];
	std::int32_t numJoints;
	std::int32_t numContacts;

	[[nodiscard]] KernelData* kernel() const noexcept;

	static constexpr float fallenY = -500.0F;
};

static_assert(offsetof(World, jointStage) == 0x34);
static_assert(offsetof(World, worldStepId) == 0x54);
static_assert(offsetof(World, primitives) == 0x58);
static_assert(offsetof(World, numJoints) == 0x70);

bool bindConstants(const Target& target);
bool bind(const Target& target);

[[nodiscard]] std::int32_t nextStateIndex() noexcept;
void updateSimBody(SimBody* simBody) noexcept;
void updateCofm(Cofm* cofm) noexcept;
void accumulateForce(SimBody* simBody, const Vector3& force, const Vector3& worldPos) noexcept;
void destroyJoints(const Array<Primitive*>& found, float reach) noexcept;
void processClumps(World* world) noexcept;
void destroyJoint(World* world, Joint* joint) noexcept;
void notifyMoved(IMoving* owner) noexcept;
void onPrimitiveTouched(World* world, Primitive* p0, Primitive* p1) noexcept;
void primitiveExtentsChanged(World* world, Primitive* primitive) noexcept;
void resizeArray(void* array, int size) noexcept;

float worldDt() noexcept;
int worldStepsPerSec() noexcept;

} // namespace v8patch::rbx
