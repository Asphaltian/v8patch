#pragma once

#include <algorithm>
#include <cmath>

#include "rbx/constants.h"

namespace v8patch::sim {

[[nodiscard]] inline int getStepCount(float desiredInterval) noexcept
{
	const float rate = static_cast<float>(rbx::Constants::worldStepsPerSec());

	return std::max(1, static_cast<int>(std::floor(rate * desiredInterval)));
}

class Clock
{
public:
	[[nodiscard]] static constexpr float budget() noexcept { return rbx::Constants::uiDt(); }

	[[nodiscard]] int take(float desiredInterval, float elapsed) noexcept
	{
		if (desiredInterval > 0.0F) {
			return getStepCount(desiredInterval);
		}

		const float rate = static_cast<float>(rbx::Constants::worldStepsPerSec());

		pending_ = std::min(pending_ + std::max(0.0F, elapsed), budget());

		const int steps = static_cast<int>(std::floor(rate * pending_));

		pending_ = std::max(0.0F, pending_ - static_cast<float>(steps) * rbx::Constants::worldDt());

		return steps;
	}

	[[nodiscard]] float pending() const noexcept { return pending_; }

private:
	float pending_ = 0.0F;
};

} // namespace v8patch::sim
