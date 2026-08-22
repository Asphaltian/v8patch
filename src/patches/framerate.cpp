#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "config.h"
#include "frame.h"
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
constexpr float kUncapped = 0.0F;

bool g_atUiStep = false;
Frame g_uiFrame;
UiStep g_uiStep;

void advanceUiStep() noexcept
{
	const float frame = g_uiFrame.elapsed();

	g_atUiStep = g_uiStep.ready(frame, frame);
}

bool atUiStep() noexcept
{
	return g_atUiStep;
}

float uiStepPhase() noexcept
{
	return std::min(1.0F, g_uiStep.waited() / rbx::Constants::uiDt());
}

namespace runservice {

using ConstructFn = void*(__fastcall*)(void*, void*);
using InvalidateFn = void(__fastcall*)(void*, void*);
using RunProcFn = void(__fastcall*)(void*, void*, void*, void*);
using RaiseFn = void(__fastcall*)(void*, void*, float, float);

std::atomic<float> g_framePeriod{1.0F / static_cast<float>(kDefaultFps)};
std::atomic<bool> g_announced{false};

ConstructFn g_construct = nullptr;
InvalidateFn g_invalidate = nullptr;
RunProcFn g_runProc = nullptr;
RaiseFn g_raiseStepped = nullptr;

Pacer g_stepped;

void configure()
{
	int fps = config::integer(kSection, "fps", kDefaultFps);

	if (fps == 0) {
		g_framePeriod.store(kUncapped, std::memory_order_relaxed);
		log::info("frame rate uncapped");
		return;
	}

	if (fps < 0) {
		log::warn("fps {} makes no sense, using {}", fps, kDefaultFps);
		fps = kDefaultFps;
	}

	g_framePeriod.store(1.0F / static_cast<float>(fps), std::memory_order_relaxed);
	log::info("target {} fps", fps);
}

void impose(void* service)
{
	if (service == nullptr) {
		return;
	}

	auto* slot = reinterpret_cast<float*>(static_cast<std::byte*>(service) + offsets::RunService::kFramePeriod);
	const float wanted = g_framePeriod.load(std::memory_order_relaxed);

	if (*slot == wanted) {
		return;
	}

	const float was = *slot;
	*slot = wanted;

	if (!g_announced.exchange(true, std::memory_order_relaxed)) {
		log::info("{:.0f} fps -> {:.0f} fps", was > 0.0F ? 1.0F / was : 0.0F, 1.0F / wanted);
	}
}

void* __fastcall onConstruct(void* service, void* edx)
{
	void* result = g_construct(service, edx);
	impose(service);

	return result;
}

void __fastcall onInvalidateRunViews(void* service, void* edx)
{
	impose(service);
	g_invalidate(service, edx);
}

void __fastcall onRunProc(void* service, void* edx, void* dataModel, void* holder)
{
	impose(service);
	g_runProc(service, edx, dataModel, holder);
}

void __fastcall onRaiseStepped(void* service, void* edx, float elapsed, float step)
{
	if (g_stepped.ready(step)) {
		g_raiseStepped(service, edx, elapsed, g_stepped.delivered());
	}
}

bool install(HookSet& hooks)
{
	using namespace offsets::RunService;

	return hooks.install(kConstructor, &onConstruct, g_construct) &&
	       hooks.install(kInvalidateRunViews, &onInvalidateRunViews, g_invalidate) &&
	       hooks.install(kRunProc, &onRunProc, g_runProc) &&
	       hooks.install(kRaiseStepped, &onRaiseStepped, g_raiseStepped);
}

} // namespace runservice

namespace scriptcontext {

using HeartbeatFn = void(__fastcall*)(void*, void*, void*, float, float);

HeartbeatFn g_onHeartbeat = nullptr;

Pacer g_heartbeat;

void __fastcall onHeartbeat(void* self, void* edx, void* service, float elapsed, float step)
{
	if (g_heartbeat.ready(step)) {
		g_onHeartbeat(self, edx, service, elapsed, g_heartbeat.delivered());
	}
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::ScriptContext::kOnHeartbeat, &onHeartbeat, g_onHeartbeat);
}

} // namespace scriptcontext

namespace shoottool {

using StepFn = void(__fastcall*)(void*, void*, void*);

StepFn g_step = nullptr;

Pacer g_reload;

void __fastcall onStep(void* tool, void* edx, void* mouse)
{
	if (g_reload.ready()) {
		g_step(tool, edx, mouse);
	}
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::ShootTool::kStep, &onStep, g_step);
}

} // namespace shoottool

namespace forcefield {

using StepFn = void(__fastcall*)(void*, void*, void*);

StepFn g_step = nullptr;

Pacer g_flash;

void __fastcall onStep(void* self, void* edx, void* part)
{
	auto* phase = reinterpret_cast<int*>(static_cast<std::byte*>(self) + offsets::ForceField::kPhase);
	const int held = *phase;
	const bool advance = g_flash.ready();

	g_step(self, edx, part);

	if (!advance) {
		*phase = held;
	}
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::ForceField::kStep, &onStep, g_step);
}

} // namespace forcefield

namespace humanoid {

using WalkFn = void(__fastcall*)(void*, void*, const rbx::Vector3*);

WalkFn g_setWalkDirection = nullptr;

struct Walk
{
	void* humanoid = nullptr;
	rbx::Vector3 direction;
};

Walk g_walks[16];

Walk* find(void* humanoid) noexcept
{
	for (Walk& walk : g_walks) {
		if (walk.humanoid == humanoid) {
			return &walk;
		}

		if (walk.humanoid == nullptr) {
			walk.humanoid = humanoid;
			return &walk;
		}
	}

	return nullptr;
}

void __fastcall onSetWalkDirection(void* self, void* edx, const rbx::Vector3* direction)
{
	Walk* walk = find(self);

	if (walk == nullptr) {
		g_setWalkDirection(self, edx, direction);
		return;
	}

	if (atUiStep()) {
		walk->direction = *direction;
	}

	g_setWalkDirection(self, edx, &walk->direction);

	std::memcpy(
	    &walk->direction, static_cast<std::byte*>(self) + offsets::Humanoid::kWalkDirection, sizeof(walk->direction)
	);
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::Humanoid::kSetWalkDirection, &onSetWalkDirection, g_setWalkDirection);
}

} // namespace humanoid

namespace character {

using UpdateFn = void(__fastcall*)(void*, void*, void*, void*);

UpdateFn g_update = nullptr;

void __fastcall onUpdate(void* subject, void* edx, void* goal, void* focus)
{
	if (atUiStep()) {
		g_update(subject, edx, goal, focus);
	}
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::ICharacterSubject::kUpdate, &onUpdate, g_update);
}

} // namespace character

namespace camera {

using UpdateGoalFn = void(__fastcall*)(void*, void*);
using KeyMoveFn = void(__fastcall*)(void*, void*, const char*, int);
using ApplyGoalFn = void(__fastcall*)(void*, void*);
using LerpFn = void*(__fastcall*)(void*, void*, void*, const float*, float);

UpdateGoalFn g_updateGoal = nullptr;
KeyMoveFn g_keyMove = nullptr;
ApplyGoalFn g_applyGoal = nullptr;
LerpFn g_lerp = nullptr;

Frame g_moved;
KeyHold g_held;

rbx::CoordinateFrame g_fromGoal;
rbx::CoordinateFrame g_toGoal;
bool g_primed = false;

void advanceGoal(const void* self) noexcept
{
	g_fromGoal = g_toGoal;
	std::memcpy(&g_toGoal, static_cast<const std::byte*>(self) + offsets::Camera::kGoal, sizeof(g_toGoal));

	if (!g_primed) {
		g_fromGoal = g_toGoal;
		g_primed = true;
	}
}

void snapGoal(const void* self) noexcept
{
	advanceGoal(self);
	g_fromGoal = g_toGoal;
}

rbx::Vector3 translation(const void* camera, std::uintptr_t offset) noexcept
{
	rbx::Vector3 value{};
	std::memcpy(&value, static_cast<const std::byte*>(camera) + offset, sizeof(value));

	return value;
}

rbx::Matrix3 rotation(const void* camera, std::uintptr_t offset) noexcept
{
	rbx::Matrix3 value{};
	std::memcpy(&value, static_cast<const std::byte*>(camera) + offset, sizeof(value));

	return value;
}

void setRotation(void* camera, std::uintptr_t offset, const rbx::Matrix3& value) noexcept
{
	std::memcpy(static_cast<std::byte*>(camera) + offset, &value, sizeof(value));
}

void keepShare(void* camera, std::uintptr_t offset, const rbx::Vector3& before, float share) noexcept
{
	const rbx::Vector3 after = translation(camera, offset);
	const rbx::Vector3 value{
	    before.x + (after.x - before.x) * share,
	    before.y + (after.y - before.y) * share,
	    before.z + (after.z - before.z) * share,
	};

	std::memcpy(static_cast<std::byte*>(camera) + offset, &value, sizeof(value));
}

void __fastcall onKeyMove(void* self, void* edx, const char* keys, int calls)
{
	using namespace offsets::Camera;

	if (calls == 0) {
		g_held.release();
	}

	const float share = g_held.advance(g_moved.elapsed());

	const rbx::Matrix3 goalRotation = rotation(self, kGoal);
	const rbx::Vector3 goal = translation(self, kGoalTranslation);
	const rbx::Vector3 focus = translation(self, kFocusTranslation);

	g_keyMove(self, edx, keys, g_held.calls());

	setRotation(self, kGoal, goalRotation);
	keepShare(self, kGoalTranslation, goal, share);
	keepShare(self, kFocusTranslation, focus, share);

	g_applyGoal(self, edx);
	snapGoal(self);
}

void __fastcall onUpdateGoal(void* self, void* edx)
{
	advanceUiStep();

	if (!atUiStep() && g_primed) {
		return;
	}

	g_updateGoal(self, edx);
	advanceGoal(self);
}

void* __fastcall onLerp(void* from, void* edx, void* out, const float* goal, float fraction)
{
	if (!g_primed) {
		return g_lerp(from, edx, out, goal, fraction);
	}

	return g_lerp(&g_fromGoal, edx, out, reinterpret_cast<const float*>(&g_toGoal), uiStepPhase());
}

bool install(HookSet& hooks, const Target& target)
{
	using namespace offsets::Camera;

	if (!target.matches(kApplyGoal)) {
		log::error("{} signature mismatch at +{:#x}", kApplyGoal.name, kApplyGoal.offset);
		return false;
	}

	g_applyGoal = reinterpret_cast<ApplyGoalFn>(target.rva(kApplyGoal.offset));

	return hooks.install(kUpdateGoal, &onUpdateGoal, g_updateGoal) && hooks.install(kKeyMove, &onKeyMove, g_keyMove) &&
	       hooks.install(offsets::CoordinateFrame::kLerp, &onLerp, g_lerp);
}

} // namespace camera

namespace world {

using StepFn = float(__fastcall*)(void*, void*, float);

StepFn g_step = nullptr;

Frame g_frame;
double g_pending = 0.0;

float __fastcall onStep(void* self, void* edx, float)
{
	const double rate = rbx::worldStepsPerSec();

	g_pending = std::min(g_pending + g_frame.elapsed(), static_cast<double>(rbx::Constants::uiDt()));

	const int steps = static_cast<int>(rate * g_pending);

	if (steps < 1) {
		return 0.0F;
	}

	g_pending -= static_cast<double>(steps) / rate;

	return g_step(self, edx, static_cast<float>((steps + 0.5) / rate));
}

bool install(HookSet& hooks)
{
	return hooks.install(offsets::World::kStep, &onStep, g_step);
}

} // namespace world

} // namespace

bool installFramerate(const Target& target)
{
	if (!rbx::bindConstants(target)) {
		return false;
	}

	runservice::configure();

	HookSet hooks(target);

	const bool installed = runservice::install(hooks) && scriptcontext::install(hooks) && shoottool::install(hooks) &&
	                       forcefield::install(hooks) && humanoid::install(hooks) && character::install(hooks) &&
	                       camera::install(hooks, target);

	if (!installed) {
		return false;
	}

	if (!ownsWorldStep() && !world::install(hooks)) {
		return false;
	}

	log::info("{} hooks installed", hooks.count());
	return true;
}

} // namespace v8patch
