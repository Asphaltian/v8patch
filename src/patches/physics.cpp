#include <windows.h>

#include <cstddef>
#include <cstring>
#include <string_view>

#include "config.h"
#include "hook.h"
#include "log.h"
#include "offsets.h"
#include "patch.h"
#include "rbx/constants.h"
#include "rbx/engine.h"
#include "sim/simulation.h"

namespace v8patch {
namespace {

constexpr std::string_view kSection = "physics";

using WorldStepFn = float(__fastcall*)(void*, void*, float);
using ComputeFallenFn = void(__fastcall*)(void*, void*, void*);
using DoBlastFn = void(__fastcall*)(void*, void*, rbx::Array<rbx::Primitive*>*);
using RemovePrimitiveFn = void(__fastcall*)(void*, void*, rbx::Primitive*);

sim::Simulation g_simulation;
WorldStepFn g_worldStep = nullptr;
ComputeFallenFn g_computeFallen = nullptr;
DoBlastFn g_doBlast = nullptr;
RemovePrimitiveFn g_removePrimitive = nullptr;

float __fastcall onWorldStep(void* self, void*, float desiredInterval)
{
	return g_simulation.step(static_cast<rbx::World*>(self), desiredInterval);
}

void __fastcall onComputeFallen(void* self, void*, void* fallen)
{
	g_simulation.update(static_cast<rbx::World*>(self));
	g_simulation.computeFallen(*static_cast<rbx::Array<rbx::Primitive*>*>(fallen));
}

void __fastcall onRemovePrimitive(void* self, void* edx, rbx::Primitive* primitive)
{
	g_simulation.invalidate();
	g_removePrimitive(self, edx, primitive);
}

template <typename T>
T read(const void* object, std::uintptr_t at) noexcept
{
	T value{};
	std::memcpy(&value, static_cast<const std::byte*>(object) + at, sizeof(value));

	return value;
}

void __fastcall onDoBlast(void* self, void*, rbx::Array<rbx::Primitive*>* found)
{
	using namespace offsets::Explosion;

	sim::Explosion explosion{};
	explosion.position = read<rbx::Vector3>(self, kPosition);
	explosion.blastRadius = read<float>(self, kBlastRadius);
	explosion.blastPressure = read<float>(self, kBlastPressure);

	if (!(explosion.blastPressure > 0.0F) || found->size() < 1) {
		return;
	}

	rbx::World* world = found->items[0]->world;

	if (world == nullptr) {
		return;
	}

	g_simulation.doBreakJoints(world);

	rbx::destroyJoints(*found, explosion.blastRadius + explosion.blastRadius);

	g_simulation.invalidate();
	g_simulation.update(world);
	g_simulation.doBlast(explosion, *found);
}

} // namespace

bool ownsWorldStep()
{
	return config::boolean(kSection, "enabled", true);
}

bool installPhysics(const Target& target)
{
	if (!rbx::bind(target) || !g_simulation.open()) {
		return false;
	}

	HookSet hooks(target);

	if (!hooks.install(offsets::World::kStep, &onWorldStep, g_worldStep) ||
	    !hooks.install(offsets::World::kComputeFallen, &onComputeFallen, g_computeFallen) ||
	    !hooks.install(offsets::Explosion::kDoBlast, &onDoBlast, g_doBlast) ||
	    !hooks.install(offsets::World::kRemovePrimitive, &onRemovePrimitive, g_removePrimitive)) {
		g_simulation.close();
		return false;
	}

	log::info("engine world pipeline retired, {} hooks installed", hooks.count());
	return true;
}

} // namespace v8patch
