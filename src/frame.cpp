#include "frame.h"

#include <windows.h>

namespace v8patch {

namespace {

double tick() noexcept
{
	static const double secondsPerCount = [] {
		LARGE_INTEGER frequency{};

		return QueryPerformanceFrequency(&frequency) != 0 && frequency.QuadPart != 0
		           ? 1.0 / static_cast<double>(frequency.QuadPart)
		           : 0.0;
	}();

	LARGE_INTEGER count{};
	QueryPerformanceCounter(&count);

	return static_cast<double>(count.QuadPart) * secondsPerCount;
}

} // namespace

float Frame::elapsed() noexcept
{
	const double entered = tick();
	const double frame = entered_ > 0.0 ? entered - entered_ : 0.0;

	entered_ = entered;

	return static_cast<float>(std::clamp(frame, 0.0, static_cast<double>(rbx::Constants::uiDt())));
}

} // namespace v8patch
