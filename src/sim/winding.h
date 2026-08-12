#pragma once

#include <box3d/box3d.h>

namespace v8patch::sim {

class Winding
{
public:
	static constexpr float threshold() noexcept { return B3_PI / 2.0F; }

	[[nodiscard]] float track(float angle) noexcept
	{
		if (!seeded_) {
			seeded_ = true;
			last_ = angle;
		} else if (last_ > threshold() && angle < -threshold()) {
			++turns_;
		} else if (last_ < -threshold() && angle > threshold()) {
			--turns_;
		}

		last_ = angle;

		return static_cast<float>(turns_) * 2.0F * B3_PI + angle;
	}

	[[nodiscard]] bool seeded() const noexcept { return seeded_; }
	[[nodiscard]] int turns() const noexcept { return turns_; }
	[[nodiscard]] float last() const noexcept { return last_; }

private:
	float last_ = 0.0F;
	int turns_ = 0;
	bool seeded_ = false;
};

} // namespace v8patch::sim
