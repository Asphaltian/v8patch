#pragma once

#include <algorithm>

#include "rbx/constants.h"

namespace v8patch {

class Frame
{
public:
	[[nodiscard]] float elapsed() noexcept;

private:
	double entered_ = 0.0;
};

class UiStep
{
public:
	[[nodiscard]] bool ready(float elapsed, float step) noexcept
	{
		waited_ += std::max(0.0F, elapsed);
		pending_ += step;

		if (waited_ < rbx::Constants::uiDt()) {
			return false;
		}

		waited_ = std::min(waited_ - rbx::Constants::uiDt(), rbx::Constants::uiDt());
		delivered_ = pending_;
		pending_ = 0.0F;

		return true;
	}

	[[nodiscard]] float delivered() const noexcept { return delivered_; }
	[[nodiscard]] float pending() const noexcept { return pending_; }
	[[nodiscard]] float waited() const noexcept { return waited_; }

private:
	float waited_ = 0.0F;
	float pending_ = 0.0F;
	float delivered_ = 0.0F;
};

class Pacer
{
public:
	[[nodiscard]] bool ready(float step) noexcept { return step_.ready(frame_.elapsed(), step); }

	[[nodiscard]] bool ready() noexcept
	{
		const float frame = frame_.elapsed();

		return step_.ready(frame, frame);
	}

	[[nodiscard]] float delivered() const noexcept { return step_.delivered(); }

private:
	Frame frame_;
	UiStep step_;
};

class KeyHold
{
public:
	void release() noexcept { held_ = 0.0F; }

	[[nodiscard]] float advance(float elapsed) noexcept
	{
		const float frame = std::max(0.0F, elapsed);

		held_ += frame;

		return std::min(1.0F, frame / rbx::Constants::uiDt());
	}

	[[nodiscard]] int calls() const noexcept { return static_cast<int>(held_ / rbx::Constants::uiDt()); }

private:
	float held_ = 0.0F;
};

} // namespace v8patch
