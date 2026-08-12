#include "sim/simulation.h"

#include <algorithm>
#include <cmath>

#include "log.h"
#include "rbx/constants.h"
#include "sim/contacts.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/tracking.h"
#include "sim/winding.h"

namespace v8patch::sim {
namespace {

constexpr int kGlueCorners = 4;
constexpr float kAxlePointOffset = 1.0F;
constexpr float kMarkerReach = 10.0F;
constexpr float kHingeDampingRatio = 2.0F;
constexpr float kTorqueArmScale = 0.1F;
constexpr float kSpringDampingRatio = 1.0F;
constexpr float kServoLag = B3_PI;
constexpr float kControllerDeadZone = 0.1F;

rbx::NormalId getNormalId(const rbx::Matrix3& rotation, bool oriented) noexcept
{
	const float x = rotation.row[2];
	const float y = rotation.row[5];
	const float z = rotation.row[8];

	const float ax = std::fabs(x);
	const float ay = std::fabs(y);
	const float az = std::fabs(z);

	if (ax >= ay && ax >= az) {
		return oriented && x < 0.0F ? rbx::NormalId::XNeg : rbx::NormalId::X;
	}

	if (ay >= az) {
		return oriented && y < 0.0F ? rbx::NormalId::YNeg : rbx::NormalId::Y;
	}

	return oriented && z < 0.0F ? rbx::NormalId::ZNeg : rbx::NormalId::Z;
}

float getPerpendicularExtent(const rbx::Primitive* primitive, rbx::NormalId normalId) noexcept
{
	const rbx::Geometry* geometry = primitive->geometry;

	if (geometry == nullptr) {
		return 0.0F;
	}

	const rbx::Vector3 gridSize = geometry->gridSize;
	const std::array<float, 3> extent{gridSize.x, gridSize.y, gridSize.z};

	const std::size_t axis = static_cast<std::size_t>(normalId) % 3;

	return std::max(extent[(axis + 1) % 3], extent[(axis + 2) % 3]);
}

float getTorqueArmLength(const rbx::Joint* joint, const rbx::Primitive* axle, const rbx::Primitive* hole) noexcept
{
	const rbx::NormalId axleId = getNormalId(joint->jointCoord0.rotation, true);
	const rbx::NormalId holeId = getNormalId(joint->jointCoord1.rotation, true);

	return std::min(getPerpendicularExtent(axle, axleId), getPerpendicularExtent(hole, holeId)) * kTorqueArmScale;
}

float getJointK(const rbx::Primitive* axle, const rbx::Primitive* hole) noexcept
{
	return std::min(rbx::jointK(axle), rbx::jointK(hole));
}

float getKmsMaxJointForce(const rbx::Joint* joint) noexcept
{
	const auto* glue = static_cast<const rbx::GlueJoint*>(joint);

	return rbx::Units::kmsForceToRbx(
	    rbx::Constants::kmsMaxJointForce(glue->overlapInP0.width(), glue->overlapInP0.height())
	);
}

float getOverlapRadius(const rbx::Joint* joint) noexcept
{
	const auto& corner = static_cast<const rbx::GlueJoint*>(joint)->overlapInP0.corner;

	constexpr float share = 1.0F / static_cast<float>(kGlueCorners);

	rbx::Vector3 centre{};

	for (const rbx::Vector3& point : corner) {
		centre.x += point.x * share;
		centre.y += point.y * share;
		centre.z += point.z * share;
	}

	float radius = 0.0F;

	for (const rbx::Vector3& point : corner) {
		radius = std::max(radius, b3Length(b3Vec3{point.x - centre.x, point.y - centre.y, point.z - centre.z}));
	}

	return radius;
}

float getHingeHertz(const Assembly& axle, const Assembly& hole, const rbx::Joint* joint) noexcept
{
	const float stiffness = getJointK(joint->prim0, joint->prim1);

	const float invMass = (axle.bodyType == b3_dynamicBody ? 1.0F / b3Body_GetMass(axle.id) : 0.0F) +
	                      (hole.bodyType == b3_dynamicBody ? 1.0F / b3Body_GetMass(hole.id) : 0.0F);

	if (!(stiffness > 0.0F) || !(invMass > 0.0F)) {
		return static_cast<float>(rbx::Constants::worldStepsPerSec()) / 8.0F;
	}

	return std::sqrt(stiffness * invMass) / (2.0F * B3_PI);
}

float axialInertia(b3BodyId body, const b3Vec3& axis) noexcept
{
	if (b3Body_GetType(body) != b3_dynamicBody) {
		return 0.0F;
	}

	const b3Vec3 local = b3InvRotateVector(b3Body_GetRotation(body), axis);

	return b3Dot(local, b3MulMV(b3Body_GetLocalRotationalInertia(body), local));
}

float getControllerValue(const rbx::Primitive* axle, const rbx::Joint* joint, int uiStepId) noexcept
{
	const rbx::SurfaceData* surfaceData =
	    axle->surfaceData[static_cast<std::size_t>(getNormalId(joint->jointCoord0.rotation, true))];

	if (surfaceData == nullptr) {
		return 0.0F;
	}

	const float paramA = surfaceData->paramA;
	const float paramB = surfaceData->paramB;
	const float time = rbx::Constants::uiDt() * static_cast<float>(uiStepId);

	switch (surfaceData->inputType) {
	case rbx::Controller::Constant:
		return paramB;
	case rbx::Controller::Sin:
		return std::sin(time * paramB) * paramA;
	case rbx::Controller::NoInput:
		return 0.0F;
	default:
		break;
	}

	const rbx::Controller* controller = axle->controller;

	if (controller == nullptr) {
		return 0.0F;
	}

	const float value = controller->value(surfaceData->inputType);

	switch (surfaceData->inputType) {
	case rbx::Controller::LeftTrack:
		if (value < -kControllerDeadZone) {
			return std::fabs(paramA) * value;
		}
		if (value > kControllerDeadZone) {
			return std::fabs(paramB) * value;
		}
		return 0.0F;
	case rbx::Controller::RightTrack:
		if (value < -kControllerDeadZone) {
			return -(std::fabs(paramA) * value);
		}
		if (value > kControllerDeadZone) {
			return -(std::fabs(paramB) * value);
		}
		return 0.0F;
	default:
		if (value < -kControllerDeadZone) {
			return -(value * paramA);
		}
		if (value > kControllerDeadZone) {
			return value * paramB;
		}
		return 0.0F;
	}
}

} // namespace

std::uint64_t Simulation::pairKey(const void* first, const void* second) noexcept
{
	const auto low = reinterpret_cast<std::uintptr_t>(first < second ? first : second);
	const auto high = reinterpret_cast<std::uintptr_t>(first < second ? second : first);

	return static_cast<std::uint64_t>(low) * kFnvPrime ^ static_cast<std::uint64_t>(high);
}

bool Simulation::filterShapes(b3ShapeId first, b3ShapeId second, void* context)
{
	const auto* simulation = static_cast<const Simulation*>(context);

	if (simulation == nullptr || simulation->matedPairs_.empty()) {
		return true;
	}

	return !simulation->matedPairs_.contains(pairKey(b3Shape_GetUserData(first), b3Shape_GetUserData(second)));
}

void Simulation::insertConnectors()
{
	connectorsPending_ = false;
	++connectorPass_;

	matedPairs_.clear();

	for (auto& held : assemblies_) {
		held->jointed = false;
	}

	for (rbx::Joint* joint : connectorJoints_) {
		insertConnector(joint, joint->jointType());
	}

	for (auto it = connectors_.begin(); it != connectors_.end();) {
		if (it->second.pass != connectorPass_) {
			removeConnector(it->second);
			it = connectors_.erase(it);
			continue;
		}

		if (isRotateJoint(it->second.jointType)) {
			matedPairs_.insert(pairKey(it->second.joint->prim0, it->second.joint->prim1));
		}

		for (rbx::Primitive* end : {it->second.joint->prim0, it->second.joint->prim1}) {
			const AssemblyPrimitive* part = getAssemblyPrimitive(end);

			if (part == nullptr || part->assembly == nullptr) {
				continue;
			}

			part->assembly->jointed = true;
		}

		++it;
	}
}

void Simulation::insertConnector(rbx::Joint* joint, rbx::Joint::Type jointType)
{
	if (!isRotateJoint(jointType) && jointType != rbx::Joint::Glue) {
		return;
	}

	const AssemblyPrimitive* axle = getAssemblyPrimitive(joint->prim0);
	const AssemblyPrimitive* hole = getAssemblyPrimitive(joint->prim1);

	if (axle == nullptr || hole == nullptr || axle->assembly == hole->assembly) {
		return;
	}

	const auto known = connectors_.find(joint);

	if (known != connectors_.end()) {
		Connector& held = known->second;

		const bool sameBodies =
		    B3_ID_EQUALS(held.bodyA, axle->assembly->id) && B3_ID_EQUALS(held.bodyB, hole->assembly->id);
		const bool mateLives = B3_IS_NULL(held.mate) || b3Joint_IsValid(held.mate);

		if (sameBodies && b3Joint_IsValid(held.id) && mateLives) {
			held.pass = connectorPass_;
			return;
		}

		removeConnector(held);
		connectors_.erase(known);
	}

	if (axle->assembly->bodyType != b3_dynamicBody && hole->assembly->bodyType != b3_dynamicBody) {
		return;
	}

	Connector connector{};
	connector.joint = joint;
	connector.axle = joint->prim0;
	connector.jointType = jointType;
	connector.pass = connectorPass_;
	connector.bodyA = axle->assembly->id;
	connector.bodyB = hole->assembly->id;

	b3Transform localFrameA = toTransform(axle->meInAssembly * joint->jointCoord0);
	b3Transform localFrameB = toTransform(hole->meInAssembly * joint->jointCoord1);

	if (isRotateJoint(jointType)) {
		const b3Vec3 axleAxis = b3RotateVector(localFrameA.q, b3Vec3_axisZ);
		const b3Vec3 holeAxis = b3RotateVector(localFrameB.q, b3Vec3_axisZ);
		const float hertz = getHingeHertz(*axle->assembly, *hole->assembly, joint);

		for (int end = 0; end < 2; ++end) {
			const float offset = end == 0 ? -kAxlePointOffset : kAxlePointOffset;

			b3SphericalJointDef def = b3DefaultSphericalJointDef();
			def.base.bodyIdA = axle->assembly->id;
			def.base.bodyIdB = hole->assembly->id;
			def.base.localFrameA.p = b3MulAdd(localFrameA.p, offset, axleAxis);
			def.base.localFrameB.p = b3MulAdd(localFrameB.p, offset, holeAxis);
			def.base.constraintHertz = hertz;
			def.base.constraintDampingRatio = kHingeDampingRatio;
			def.base.collideConnected = true;

			(end == 0 ? connector.id : connector.mate) = b3CreateSphericalJoint(world_, &def);
		}

		connector.baseInAxle = b3MulAdd(localFrameA.p, -kAxlePointOffset, axleAxis);
		connector.rayInAxle = b3MulAdd(localFrameA.p, kAxlePointOffset, axleAxis);

		if (jointType != rbx::Joint::Rotate) {
			const float arm = getTorqueArmLength(joint, joint->prim0, joint->prim1);

			connector.servoK = getJointK(joint->prim0, joint->prim1) * arm * arm;
			connector.markerInAxle = b3MulAdd(localFrameA.p, kMarkerReach, b3RotateVector(localFrameA.q, b3Vec3_axisX));
			connector.markerInHole = b3MulAdd(localFrameB.p, kMarkerReach, b3RotateVector(localFrameB.q, b3Vec3_axisX));
		}
	} else {
		const bool holeMoves = hole->assembly->bodyType != b3_staticBody;

		b3WeldJointDef def = b3DefaultWeldJointDef();
		def.base.bodyIdA = holeMoves ? axle->assembly->id : hole->assembly->id;
		def.base.bodyIdB = holeMoves ? hole->assembly->id : axle->assembly->id;
		def.base.localFrameA = holeMoves ? localFrameA : localFrameB;
		def.base.localFrameB = holeMoves ? localFrameB : localFrameA;
		def.base.collideConnected = true;

		connector.id = b3CreateWeldJoint(world_, &def);

		const float perCorner = getKmsMaxJointForce(joint);

		if (perCorner > 0.0F && std::isfinite(perCorner)) {
			b3Joint_SetForceThreshold(connector.id, perCorner * kGlueCorners);
			b3Joint_SetTorqueThreshold(connector.id, perCorner * 2.0F * std::max(1.0F, getOverlapRadius(joint)));
		}
	}

	const auto entry = connectors_.emplace(joint, connector).first;
	b3Joint_SetUserData(entry->second.id, &entry->second);
}

void Simulation::computeForce(Connector& connector, int uiStepId)
{
	if (!(connector.servoK > 0.0F) || !b3Joint_IsValid(connector.id)) {
		return;
	}

	const b3BodyId axle = b3Joint_GetBodyA(connector.id);
	const b3BodyId hole = b3Joint_GetBodyB(connector.id);

	const b3Vec3 base = b3Body_GetWorldPoint(axle, connector.baseInAxle);
	const b3Vec3 ray = b3Body_GetWorldPoint(axle, connector.rayInAxle);

	b3Vec3 normal = b3Sub(ray, base);
	const float span = b3Length(normal);

	if (!(span > 0.0F)) {
		return;
	}

	normal = b3MulSV(1.0F / span, normal);

	const b3Vec3 rayRef0 = b3Sub(b3Body_GetWorldPoint(axle, connector.markerInAxle), base);
	const b3Vec3 rayRef1 = b3Sub(b3Body_GetWorldPoint(hole, connector.markerInHole), base);

	const b3Vec3 inPlane0 = b3MulAdd(rayRef0, -b3Dot(rayRef0, normal), normal);
	const b3Vec3 inPlane1 = b3MulAdd(rayRef1, -b3Dot(rayRef1, normal), normal);

	const float angle = std::atan2(b3Dot(b3Cross(inPlane0, inPlane1), normal), b3Dot(inPlane1, inPlane0));

	if (!connector.winding.seeded()) {
		connector.goal = angle;
	}

	const float rotation = connector.winding.track(angle);

	const float channel = getControllerValue(connector.axle, connector.joint, uiStepId);

	if (connector.jointType != rbx::Joint::RotateV && channel == 0.0F) {
		return;
	}

	const float spin = axialInertia(axle, normal);
	const float carry = axialInertia(hole, normal);
	const float share = spin + carry;

	if (!(share > 0.0F)) {
		return;
	}

	const float effective = spin > 0.0F && carry > 0.0F ? spin * carry / share : share;

	const int inner = rbx::Constants::kernelStepsPerWorldStep();
	const float dt = rbx::Constants::kernelDt();
	const float increment = channel / static_cast<float>(rbx::Constants::kernelStepsPerUiStep());

	const b3Vec3 slip = b3Sub(b3Body_GetAngularVelocity(hole), b3Body_GetAngularVelocity(axle));

	const float was = b3Dot(slip, normal);
	float rate = was;
	float turned = rotation;

	for (int step = 0; step < inner; ++step) {
		if (connector.jointType == rbx::Joint::RotateV) {
			connector.goal = increment != 0.0F ? connector.goal + increment : (connector.goal + turned) * 0.5F;
		} else {
			connector.goal = channel;
		}

		rate += (connector.goal - turned) * connector.servoK * dt / effective;
		turned += rate * dt;
	}

	const b3Vec3 impulse = b3MulSV(effective * (rate - was), normal);

	b3Body_ApplyAngularImpulse(axle, b3Neg(impulse), false);
	b3Body_ApplyAngularImpulse(hole, impulse, true);
}

void Simulation::driveConnectors(int uiStepId)
{
	for (auto& entry : connectors_) {
		computeForce(entry.second, uiStepId);
	}
}

void Simulation::findBreakingConnectors()
{
	const b3JointEvents events = b3World_GetJointEvents(world_);

	for (int i = 0; i < events.count; ++i) {
		auto* connector = static_cast<Connector*>(events.jointEvents[i].userData);

		if (connector == nullptr || connector->broken || connector->jointType != rbx::Joint::Glue) {
			continue;
		}

		connector->broken = true;
		breakingJoints_.push_back(connector->joint);

		removeConnector(*connector);
	}
}

bool Simulation::doBreakJoints(rbx::World* world)
{
	if (breakingJoints_.empty()) {
		return false;
	}

	for (rbx::Joint* joint : breakingJoints_) {
		connectors_.erase(joint);

		if (liveJoints_.contains(joint)) {
			rbx::destroyJoint(world, joint);
		}
	}

	breakingJoints_.clear();
	connectorJoints_.clear();
	jointCount_ = -1;

	return true;
}

void Simulation::removeConnector(Connector& connector)
{
	if (b3Joint_IsValid(connector.id)) {
		b3DestroyJoint(connector.id, true);
	}

	if (b3Joint_IsValid(connector.mate)) {
		b3DestroyJoint(connector.mate, true);
	}

	connector.id = b3_nullJointId;
	connector.mate = b3_nullJointId;
}

} // namespace v8patch::sim
