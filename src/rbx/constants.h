#pragma once

#include "rbx/engine.h"

namespace v8patch::rbx {

class Constants
{
public:
	static constexpr int worldStepsPerUiStep() noexcept { return 8; }
	static constexpr int kernelStepsPerWorldStep() noexcept { return 19; }
	static constexpr int worldStepsPerSec() noexcept { return 240; }
	static constexpr int kernelStepsPerSec() noexcept { return 4560; }
	static constexpr int kernelStepsPerUiStep() noexcept { return 152; }

	static constexpr float uiDt() noexcept { return 0.033333335F; }
	static constexpr float worldDt() noexcept { return 0.004166667F; }
	static constexpr float kernelDt() noexcept { return 0.00021929825F; }

	static constexpr Vector3 kmsGravity() noexcept { return Vector3{0.0F, -9.81F, 0.0F}; }

	static constexpr float contactReferenceRetention() noexcept { return 0.999F; }

	static float contactBiasRate() noexcept;

	static constexpr float sleepTolerance() noexcept { return 0.02F; }
	static constexpr float runningAverageWeight() noexcept { return 0.25F; }

	static constexpr float sleepVelocity() noexcept
	{
		return sleepTolerance() * runningAverageWeight() / (1.0F - runningAverageWeight()) *
		       static_cast<float>(worldStepsPerSec());
	}

	static float elasticMultiplier(float elasticity) noexcept;
	static float kmsMaxJointForce(float grid1, float grid2) noexcept;
	static float jointK(const Vector3& gridSize, bool ball) noexcept;

private:
	static float jointKMultiplier(const Vector3& clippedSortedSize, bool ball) noexcept;
};

class Units
{
public:
	static constexpr float studsPerMetre = 20.0F;

	static constexpr Vector3 kmsAccelerationToRbx(const Vector3& a) noexcept
	{
		return Vector3{a.x * studsPerMetre, a.y * studsPerMetre, a.z * studsPerMetre};
	}

	static constexpr float kmsForceToRbx(float f) noexcept { return f * studsPerMetre; }
};

float jointK(const Primitive* primitive) noexcept;

} // namespace v8patch::rbx
