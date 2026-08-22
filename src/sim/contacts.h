#pragma once

#include <box3d/box3d.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rbx/constants.h"

namespace v8patch::sim {

[[nodiscard]] inline float getFriction(float first, std::uint64_t, float second, std::uint64_t) noexcept
{
	return std::min(first, second);
}

[[nodiscard]] inline float getRestitution(float first, std::uint64_t, float second, std::uint64_t) noexcept
{
	return rbx::Constants::elasticMultiplier(std::min(first, second));
}

[[nodiscard]] inline float getBiasRate(float hertz, float damping, float substepDt) noexcept
{
	const float omega = 2.0F * B3_PI * hertz;
	const float a1 = 2.0F * damping + substepDt * omega;
	const float a2 = substepDt * omega * a1;

	return a2 / (1.0F + a2) * omega / a1;
}

[[nodiscard]] inline float getContactDamping(float hertz, float substepDt) noexcept
{
	const float omega = 2.0F * B3_PI * hertz;
	const float a1 = omega / rbx::Constants::contactBiasRate() - 1.0F / (substepDt * omega);

	return std::max(0.0F, 0.5F * (a1 - substepDt * omega));
}

[[nodiscard]] inline float getStiffestHertz(float substepDt) noexcept
{
	return 0.125F / substepDt;
}

} // namespace v8patch::sim
