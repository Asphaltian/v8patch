#pragma once

#include <cmath>

#include "rbx/engine.h"
#include "sim/frames.h"

namespace v8patch::sim {

constexpr int kMaxJointsPerPrimitive = 4096;

inline bool isRigidJoint(rbx::Joint::Type jointType) noexcept
{
	return jointType == rbx::Joint::Weld || jointType == rbx::Joint::Snap || jointType == rbx::Joint::Motor;
}

inline bool isRotateJoint(rbx::Joint::Type jointType) noexcept
{
	return jointType == rbx::Joint::Rotate || jointType == rbx::Joint::RotateP || jointType == rbx::Joint::RotateV;
}

inline bool isConnectorJoint(rbx::Joint::Type jointType) noexcept
{
	return isRotateJoint(jointType) || jointType == rbx::Joint::Glue;
}

inline rbx::CoordinateFrame computeChildInParent(const rbx::Joint* joint, bool parentIsPrim0, float angle) noexcept
{
	const rbx::CoordinateFrame& parentCoord = joint->jointCoord(parentIsPrim0 ? 0 : 1);
	const rbx::CoordinateFrame& childCoord = joint->jointCoord(parentIsPrim0 ? 1 : 0);

	return rbx::rotateAboutZ(parentCoord, angle) * rbx::inverse(childCoord);
}

inline float stepUiAngle(rbx::MotorJoint* motor) noexcept
{
	const float maxStep = std::fabs(motor->maxVelocity);
	const float delta = motor->desiredAngle - motor->currentAngle;

	if (std::fabs(delta) < maxStep) {
		motor->currentAngle = motor->desiredAngle;
	} else if (delta > 0.0F) {
		motor->currentAngle += maxStep;
	} else {
		motor->currentAngle -= maxStep;
	}

	return motor->currentAngle;
}

} // namespace v8patch::sim
