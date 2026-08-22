#pragma once

#include <box3d/box3d.h>

#include <algorithm>

#include "rbx/constants.h"
#include "rbx/engine.h"
#include "sim/frames.h"
#include "sim/simulation.h"

namespace v8patch::sim {

constexpr float kLeastExtent = 0.05F;
constexpr float kLeastDensity = 1.0e-4F;

constexpr int kSubSteps = rbx::Constants::kernelStepsPerWorldStep();
constexpr float kSleepVelocity = rbx::Constants::sleepVelocity();

inline float angularDamping() noexcept
{
	return -static_cast<float>(rbx::Constants::kernelStepsPerSec()) * std::log(rbx::kAngularRetention);
}

inline rbx::SimBody* getSimBody(const rbx::Body* body) noexcept
{
	rbx::SimBody* simBody = body->simBody;

	return simBody != nullptr && simBody->body == body ? simBody : nullptr;
}

inline bool getCanCollide(const rbx::Primitive* primitive) noexcept
{
	return primitive->canCollide != 0 && primitive->dragging == 0;
}

inline b3BodyType getBodyType(const Assembly& assembly) noexcept
{
	return assembly.anchored ? b3_staticBody : b3_dynamicBody;
}

inline bool getAnchor(const rbx::Primitive* primitive) noexcept
{
	return primitive->anchored != 0 || primitive->dragging != 0;
}

inline b3BoxHull makeHull(const rbx::Vector3& gridSize, const b3Transform& at) noexcept
{
	return b3MakeTransformedBoxHull(
	    std::max(kLeastExtent, gridSize.x * 0.5F),
	    std::max(kLeastExtent, gridSize.y * 0.5F),
	    std::max(kLeastExtent, gridSize.z * 0.5F),
	    at
	);
}

} // namespace v8patch::sim
