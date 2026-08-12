#include "rbx/engine.h"

#include <cmath>
#include <cstring>

#include "log.h"
#include "offsets.h"

namespace v8patch::rbx {
namespace {

using UpdateSimBodyFn = void(__thiscall*)(SimBody*);
using AccumulateForceFn = void(__thiscall*)(SimBody*, const Vector3*, const Vector3*);
using UpdateCofmFn = void(__thiscall*)(Cofm*);
using FromPrimitiveFn = void*(__cdecl*)(const Primitive*);
using InForceFieldFn = bool(__cdecl*)(const void*);
using DestroyJointsFn = void(__thiscall*)(void*);
using NotifyMovedFn = void(__thiscall*)(IMoving*);
using TouchedFn = void(__thiscall*)(World*, Primitive*, Primitive*);
using ExtentsChangedFn = void(__thiscall*)(void*, Primitive*);
using RaiseFn = void(__thiscall*)(void*, void*);
using ResizeFn = void(__thiscall*)(void*, int, bool);

UpdateSimBodyFn g_updateSimBody = nullptr;
AccumulateForceFn g_accumulateForce = nullptr;
UpdateCofmFn g_updateCofm = nullptr;
FromPrimitiveFn g_partFromPrimitive = nullptr;
InForceFieldFn g_partInForceField = nullptr;
DestroyJointsFn g_destroyPartJoints = nullptr;
NotifyMovedFn g_notifyMoved = nullptr;
TouchedFn g_onPrimitiveTouched = nullptr;
ExtentsChangedFn g_primitiveExtentsChanged = nullptr;
RaiseFn g_raiseAutoDestroy = nullptr;
ResizeFn g_resizeArray = nullptr;

std::int32_t* g_stateIndex = nullptr;

float g_worldDt = 0.0F;
int g_worldStepsPerSec = 0;

template <typename Fn>
bool adopt(const Target& target, const offsets::Site& site, Fn& out)
{
	if (!target.matches(site)) {
		log::error("{} is not at +{:#x} in this build", site.name, site.offset);
		return false;
	}

	out = reinterpret_cast<Fn>(target.rva(site.offset));
	return true;
}

} // namespace

bool bindConstants(const Target& target)
{
	using namespace offsets::Constants;

	if (!target.matches(kWorldStepsPerSec) || !target.matches(kWorldDt)) {
		log::error("the world timestep constants are not where this build says they are");
		return false;
	}

	const auto* rate = static_cast<const std::uint8_t*>(target.rva(kWorldStepsPerSec.offset));
	const auto* step = static_cast<const std::uint8_t*>(target.rva(kWorldDt.offset));

	std::uint32_t perSecond = 0;
	std::memcpy(&perSecond, rate + kImmediateOperand, sizeof(perSecond));

	std::uintptr_t operand = 0;
	std::memcpy(&operand, step + kLoadOperand, sizeof(operand));
	std::memcpy(&g_worldDt, reinterpret_cast<const void*>(operand), sizeof(g_worldDt));

	g_worldStepsPerSec = static_cast<int>(perSecond);

	return g_worldStepsPerSec > 0 && g_worldDt > 0.0F;
}

namespace {

float distance(const Vector3& from, const Vector3& to) noexcept
{
	const float dx = to.x - from.x;
	const float dy = to.y - from.y;
	const float dz = to.z - from.z;

	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

float Face::width() const noexcept
{
	return distance(corner[0], corner[1]);
}

float Face::height() const noexcept
{
	return distance(corner[0], corner[3]);
}

KernelData* World::kernel() const noexcept
{
	const IStage* stage = jointStage;

	for (int depth = 0; stage != nullptr && depth < IStage::KernelStage + 1; ++depth) {
		if (stage->stageType() == IStage::KernelStage) {
			return reinterpret_cast<const Kernel*>(stage)->kernelData;
		}

		stage = stage->downstream;
	}

	return nullptr;
}

namespace {

bool bindStateIndex(const Target& target)
{
	if (!target.matches(offsets::Body::kStateIndex)) {
		log::error("the body state index counter is not where this build says it is");
		return false;
	}

	const auto* load = static_cast<const std::uint8_t*>(target.rva(offsets::Body::kStateIndex.offset));

	std::uintptr_t operand = 0;
	std::memcpy(&operand, load + offsets::Body::kStateIndexOperand, sizeof(operand));

	g_stateIndex = reinterpret_cast<std::int32_t*>(operand);

	return g_stateIndex != nullptr;
}

} // namespace

bool bind(const Target& target)
{
	return bindConstants(target) && bindStateIndex(target) &&
	       adopt(target, offsets::SimBody::kUpdate, g_updateSimBody) &&
	       adopt(target, offsets::SimBody::kAccumulateForce, g_accumulateForce) &&
	       adopt(target, offsets::Cofm::kUpdate, g_updateCofm) &&
	       adopt(target, offsets::PartInstance::kFromPrimitive, g_partFromPrimitive) &&
	       adopt(target, offsets::PartInstance::kDestroyJoints, g_destroyPartJoints) &&
	       adopt(target, offsets::ForceField::kPartInForceField, g_partInForceField) &&
	       adopt(target, offsets::IMoving::kNotifyMoved, g_notifyMoved) &&
	       adopt(target, offsets::World::kOnPrimitiveTouched, g_onPrimitiveTouched) &&
	       adopt(target, offsets::SpatialHash::kPrimitiveExtentsChanged, g_primitiveExtentsChanged) &&
	       adopt(target, offsets::World::kRaiseAutoDestroy, g_raiseAutoDestroy) &&
	       adopt(target, offsets::Array::kResize, g_resizeArray);
}

std::int32_t nextStateIndex() noexcept
{
	std::int32_t value = *g_stateIndex + 1;

	if (value == 0x7fffffff) {
		value = 1;
	}

	*g_stateIndex = value;

	return value;
}

void updateSimBody(SimBody* simBody) noexcept
{
	g_updateSimBody(simBody);
}

void updateCofm(Cofm* cofm) noexcept
{
	g_updateCofm(cofm);
}

void accumulateForce(SimBody* simBody, const Vector3& force, const Vector3& worldPos) noexcept
{
	g_accumulateForce(simBody, &force, &worldPos);
}

void destroyJoints(const Array<Primitive*>& found, float reach) noexcept
{
	for (Primitive* primitive : found) {
		if (!(primitive->radius() < reach)) {
			continue;
		}

		void* part = g_partFromPrimitive(primitive);

		if (part != nullptr && !g_partInForceField(part)) {
			g_destroyPartJoints(part);
		}
	}
}

void notifyMoved(IMoving* owner) noexcept
{
	if (owner != nullptr) {
		g_notifyMoved(owner);
	}
}

void onPrimitiveTouched(World* world, Primitive* p0, Primitive* p1) noexcept
{
	g_onPrimitiveTouched(world, p0, p1);
}

void primitiveExtentsChanged(World* world, Primitive* primitive) noexcept
{
	if (world->contactManager == nullptr) {
		return;
	}

	void* spatialHash = *static_cast<void**>(world->contactManager);

	if (spatialHash != nullptr) {
		g_primitiveExtentsChanged(spatialHash, primitive);
	}
}

void resizeArray(void* array, int size) noexcept
{
	g_resizeArray(array, size, false);
}

void destroyJoint(World* world, Joint* joint) noexcept
{
	auto* notifier = reinterpret_cast<std::byte*>(world) + offsets::World::kAutoDestroyNotifier;

	world->inJointNotification = 1;
	g_raiseAutoDestroy(notifier, joint);
	world->inJointNotification = 0;
}

float worldDt() noexcept
{
	return g_worldDt;
}

int worldStepsPerSec() noexcept
{
	return g_worldStepsPerSec;
}

} // namespace v8patch::rbx
