#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <string_view>

#include "config.h"
#include "hook.h"
#include "log.h"
#include "offsets.h"
#include "patch.h"
#include "rbx/constants.h"
#include "rbx/engine.h"

namespace v8patch {
namespace {

constexpr std::string_view kSection = "framerate";

constexpr int kDefaultFps = 60;
constexpr float kUnlimitedPeriod = 0.0F;
constexpr double kMaxCatchUp = rbx::Constants::uiDt();

namespace pacer {

using CtorFn = void*(__fastcall*)(void*, void*);
using ThisOnlyFn = void(__fastcall*)(void*, void*);
using RunProcFn = void(__fastcall*)(void*, void*, void*, void*);

std::atomic<float> g_period{1.0F / static_cast<float>(kDefaultFps)};
std::atomic<bool> g_announced{false};

CtorFn g_construct = nullptr;
ThisOnlyFn g_invalidate = nullptr;
RunProcFn g_run = nullptr;

void configure()
{
	int fps = config::integer(kSection, "fps", kDefaultFps);

	if (fps == 0) {
		g_period.store(kUnlimitedPeriod, std::memory_order_relaxed);
		log::info("frame rate uncapped");
		return;
	}

	if (fps < 0) {
		log::warn("fps {} makes no sense, using {}", fps, kDefaultFps);
		fps = kDefaultFps;
	}

	g_period.store(1.0F / static_cast<float>(fps), std::memory_order_relaxed);
	log::info("target {} fps", fps);
}

void impose(void* self)
{
	if (self == nullptr) {
		return;
	}

	auto* slot = reinterpret_cast<float*>(static_cast<std::byte*>(self) + offsets::RunService::kFramePeriod);
	const float wanted = g_period.load(std::memory_order_relaxed);

	if (*slot == wanted) {
		return;
	}

	const float was = *slot;
	*slot = wanted;

	if (!g_announced.exchange(true, std::memory_order_relaxed)) {
		log::info("{:.0f} fps -> {:.0f} fps", was > 0.0F ? 1.0F / was : 0.0F, 1.0F / wanted);
	}
}

void* __fastcall onConstruct(void* self, void* edx)
{
	void* result = g_construct(self, edx);
	impose(self);

	return result;
}

void __fastcall onInvalidate(void* self, void* edx)
{
	impose(self);
	g_invalidate(self, edx);
}

void __fastcall onRun(void* self, void* edx, void* px, void* pn)
{
	impose(self);
	g_run(self, edx, px, pn);
}

bool install(HookSet& hooks)
{
	using namespace offsets::RunService;

	return hooks.install(kConstructor, &onConstruct, g_construct) &&
	       hooks.install(kInvalidateRunViews, &onInvalidate, g_invalidate) && hooks.install(kRunProc, &onRun, g_run);
}

} // namespace pacer

namespace worldclock {

using StepFn = float(__fastcall*)(void*, void*, float);

StepFn g_step = nullptr;
double g_secondsPerTick = 0.0;
double g_previous = 0.0;
double g_carry = 0.0;

double now() noexcept
{
	LARGE_INTEGER counter{};
	QueryPerformanceCounter(&counter);

	return static_cast<double>(counter.QuadPart) * g_secondsPerTick;
}

float __fastcall onStep(void* self, void* edx, float)
{
	const double rate = rbx::worldStepsPerSec();
	const double entered = now();

	if (g_previous == 0.0) {
		g_previous = entered;
		return 0.0F;
	}

	g_carry = std::min(g_carry + std::max(0.0, entered - g_previous), kMaxCatchUp);
	g_previous = entered;

	const int steps = static_cast<int>(rate * g_carry);

	if (steps < 1) {
		return 0.0F;
	}

	g_carry -= static_cast<double>(steps) / rate;

	return g_step(self, edx, static_cast<float>((steps + 0.5) / rate));
}

bool install(HookSet& hooks)
{
	LARGE_INTEGER frequency{};

	if (QueryPerformanceFrequency(&frequency) == 0 || frequency.QuadPart == 0) {
		log::error("no performance counter to pace the world against");
		return false;
	}

	g_secondsPerTick = 1.0 / static_cast<double>(frequency.QuadPart);

	return hooks.install(offsets::World::kStep, &onStep, g_step);
}

} // namespace worldclock

namespace stepgate {

using RaiseFn = void(__fastcall*)(void*, void*, float, float);

RaiseFn g_raise = nullptr;
float g_carry = 0.0F;

void __fastcall onRaise(void* self, void* edx, float time, float step)
{
	g_carry += step;

	if (g_carry < rbx::Constants::uiDt()) {
		return;
	}

	const float delivered = g_carry;
	g_carry = 0.0F;

	g_raise(self, edx, time, delivered);
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::RunService::kRaiseStepped, &onRaise, g_raise);
}

} // namespace stepgate

} // namespace

bool installFramerate(const Target& target)
{
	if (!rbx::bindConstants(target)) {
		return false;
	}

	pacer::configure();

	HookSet hooks(target);

	if (!pacer::install(hooks) || !stepgate::install(hooks)) {
		return false;
	}

	if (!ownsWorldStep() && !worldclock::install(hooks)) {
		return false;
	}

	log::info("{} hooks installed", hooks.count());
	return true;
}

} // namespace v8patch
