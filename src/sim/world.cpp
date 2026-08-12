#include "sim/simulation.h"

#include <windows.h>

#include <algorithm>
#include <cmath>

#include "log.h"
#include "rbx/constants.h"
#include "sim/contacts.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/stepping.h"
#include "sim/tracking.h"

namespace v8patch::sim {
namespace {

int onAssert(const char* condition, const char* file, int line)
{
	static int reported = 0;

	if (reported < 32) {
		++reported;
		log::error("box3d {} failed at {}:{}", condition, file, line);
	}

	return 0;
}

} // namespace

double getTick() noexcept
{
	static const double period = [] {
		LARGE_INTEGER frequency{};
		QueryPerformanceFrequency(&frequency);
		return frequency.QuadPart != 0 ? 1.0 / static_cast<double>(frequency.QuadPart) : 0.0;
	}();

	LARGE_INTEGER counter{};
	QueryPerformanceCounter(&counter);

	return static_cast<double>(counter.QuadPart) * period;
}

bool Simulation::open()
{
	SYSTEM_INFO system{};
	GetSystemInfo(&system);

	const int workers = std::clamp(static_cast<int>(system.dwNumberOfProcessors) / 2, 1, B3_MAX_WORKERS);

	b3SetAssertFcn(&onAssert);
	b3SetLengthUnitsPerMeter(rbx::Units::studsPerMetre);

	const float substepDt = rbx::Constants::worldDt() / static_cast<float>(kSubSteps);
	const float hertz = getStiffestHertz(substepDt);
	const float damping = getContactDamping(hertz, substepDt);

	b3WorldDef def = b3DefaultWorldDef();
	def.gravity = toVec3(rbx::Units::kmsAccelerationToRbx(rbx::Constants::kmsGravity()));
	def.contactHertz = hertz;
	def.contactDampingRatio = damping;
	def.frictionCallback = &getFriction;
	def.restitutionCallback = &getRestitution;
	def.enableSleep = true;
	def.workerCount = static_cast<std::uint32_t>(workers);

	gravityStep_ = b3MulSV(rbx::Constants::worldDt(), def.gravity);
	spinRegain_ = 1.0F + substepDt * angularDamping();

	world_ = b3CreateWorld(&def);

	b3World_SetCustomFilterCallback(world_, &Simulation::filterShapes, this);

	if (!b3World_IsValid(world_)) {
		log::error("box3d refused to create a world");
		return false;
	}

	return true;
}

void Simulation::close()
{
	if (b3World_IsValid(world_)) {
		b3DestroyWorld(world_);
	}

	world_ = b3_nullWorldId;

	assemblies_.clear();
	connectors_.clear();
	primitiveToPart_.clear();
	matedPairs_.clear();
	forced_.clear();
	unresolved_.clear();

	primitiveSignature_ = 0;
	primitiveCount_ = -1;
	jointCount_ = -1;
	connectorsPending_ = false;
}

void Simulation::invalidate() noexcept
{
	primitiveSignature_ = 0;
	primitiveCount_ = -1;
	jointCount_ = -1;
}

void Simulation::update(rbx::World* world)
{
	const rbx::Array<rbx::Primitive*>& primitives = world->primitives;
	const std::int32_t count = primitives.size();

	if (count == primitiveCount_ && world->numJoints == jointCount_ && ++sinceScan_ < kScanEvery && resolved()) {
		return;
	}

	sinceScan_ = 0;

	std::uint64_t signature = 0;

	for (std::int32_t i = 0; i < count; ++i) {
		signature = signatureOf(primitives.items[i], signature);
	}

	if (signature == primitiveSignature_ && count == primitiveCount_ && world->numJoints == jointCount_ && resolved()) {
		return;
	}

	livePrimitives_.clear();
	livePrimitives_.reserve(static_cast<std::size_t>(count) * 2);

	for (std::int32_t i = 0; i < count; ++i) {
		livePrimitives_.insert(primitives.items[i]);
	}

	findClumps(primitives);
	cleanAssemblies(primitives);

	primitiveSignature_ = signature;
	primitiveCount_ = count;
	jointCount_ = world->numJoints;
	connectorsPending_ = true;
}

float Simulation::step(rbx::World* world, float desiredInterval)
{
	update(world);

	if (kernel_ == nullptr) {
		kernel_ = world->kernel();
	}

	const double startTime = getTick();
	const float worldDt = rbx::Constants::worldDt();

	const float elapsed = entered_ > 0.0 ? static_cast<float>(startTime - entered_) : 0.0F;
	entered_ = startTime;

	const int steps = clock_.take(desiredInterval, elapsed);

	bool throttling = false;
	bool notify = false;

	for (int j = 0; j < steps; ++j) {
		update(world);

		const int uiStepId = world->worldStepId / rbx::Constants::worldStepsPerUiStep();
		const bool sweeping = world->worldStepId % rbx::Constants::worldStepsPerUiStep() == 0;

		if (sweeping) {
			world->inStepCode = 1;

			if (doBreakJoints(world)) {
				update(world);
			}

			world->touch.count = 0;
			world->touchOther.count = 0;

			stepUi();
			insertConnectors();

			world->inStepCode = 0;
			notify = true;
		} else if (connectorsPending_) {
			insertConnectors();
		}

		update(world);

		world->inStepCode = 1;

		driveConnectors(uiStepId);
		updateBodies();
		stepKernel(kernel_, throttling, sweeping);

		world->inStepCode = 0;

		b3World_Step(world_, worldDt, kSubSteps);

		publishBodies(world, notify);
		reportTouches(world);
		findBreakingConnectors();

		notify = false;

		throttling = world->canThrottle != 0 && getTick() > startTime + static_cast<double>(worldDt) * (j + 1);

		world->worldStepId += 1;
	}

	return worldDt * static_cast<float>(steps);
}
} // namespace v8patch::sim
