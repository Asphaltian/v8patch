#include "check.h"
#include "world.h"

namespace v8patch::tests {
namespace {

using v8patch::rbx::Vector3;

constexpr Vector3 kStone{2.0F, 1.2F, 4.0F};
constexpr float kElasticity = 0.1F;
constexpr float kFriction = 0.85F;

float bounceRatio(float elasticity)
{
	const Bed bed{true};

	bed.part(Vector3{12.0F, 2.0F, 12.0F}, b3Vec3{0.0F, 0.0F, 0.0F}, true, elasticity, kFriction);

	const Vector3 cube{2.0F, 2.0F, 2.0F};
	const float seat = 2.0F;
	const float from = 60.0F;

	const b3BodyId id = bed.part(cube, b3Vec3{0.0F, from, 0.0F}, false, elasticity, kFriction);

	bool landed = false;
	float peak = seat;

	for (int beat = 0; beat < 240 * 8; ++beat) {
		bed.step();

		const float y = b3Body_GetPosition(id).y;

		if (!landed) {
			landed = from - y > 20.0F && b3Body_GetLinearVelocity(id).y > 1.0F;
		} else if (y > peak) {
			peak = y;
		}
	}

	return (peak - seat) / (from - seat);
}

void aContactNeverReturnsMoreThanItTook()
{
	float last = 0.0F;

	for (float elasticity : {0.0F, 0.2F, 0.5F, 0.7F, 1.0F}) {
		const float ratio = bounceRatio(elasticity);

		check(ratio > 0.0F, "a dropped part bounces");
		check(ratio < 1.0F, "a bounce never returns more height than it fell");
		check(ratio > last, "a livelier surface bounces higher than a deader one");

		last = ratio;
	}
}

void aRestingStackHoldsTheEngineSpacing()
{
	const Bed bed{true};

	bed.part(Vector3{30.0F, 2.0F, 30.0F}, b3Vec3{0.0F, 20.0F, 0.0F}, true);

	b3BodyId stack[8]{};

	for (int i = 0; i < 8; ++i) {
		stack[i] = bed.part(kStone, b3Vec3{0.0F, 21.6F + kStone.y * static_cast<float>(i), 0.0F}, false, kElasticity, kFriction);
	}

	const int slept = bed.settle(stack[7], 240 * 8);
	const float spacing = (b3Body_GetPosition(stack[7]).y - b3Body_GetPosition(stack[0]).y) / 7.0F;

	check(slept >= 0, "a stack of eight goes to sleep");
	check(spacing < kStone.y, "a settled stack beds into itself");
	near(spacing, 1.19200F, 0.01F, "a settled stack holds the spacing the engine holds");
}

void aWallSettlesAndSleeps()
{
	constexpr int kRows = 20;
	constexpr int kSpan = 24;

	const Bed bed{true};

	bed.part(Vector3{80.0F, 2.0F, 30.0F}, b3Vec3{0.0F, -1.0F, 0.0F}, true);

	static b3BodyId wall[kRows][kSpan]{};

	for (int row = 0; row < kRows; ++row) {
		for (int at = 0; at < kSpan; ++at) {
			const float bond = row % 2 == 0 ? 0.0F : kStone.x * 0.5F;
			const float x = bond + kStone.x * (static_cast<float>(at) - 0.5F * static_cast<float>(kSpan - 1));

			wall[row][at] = bed.part(kStone, b3Vec3{x, kStone.y * (0.5F + static_cast<float>(row)), 0.0F}, false, kElasticity, kFriction);
		}
	}

	const b3BodyId top = wall[kRows - 1][0];

	for (int beat = 0; beat < 240 * 6; ++beat) {
		bed.step();
	}

	const float held = b3Body_GetPosition(top).y;

	float fastest = 0.0F;
	float low = held;
	float high = held;

	for (int beat = 0; beat < 240; ++beat) {
		bed.step();

		const float y = b3Body_GetPosition(top).y;
		const b3Vec3 v = b3Body_GetLinearVelocity(top);
		const float speed = b3Length(v);

		fastest = speed > fastest ? speed : fastest;
		low = y < low ? y : low;
		high = y > high ? y : high;
	}

	check(fastest < v8patch::sim::kSleepVelocity, "a settled wall never breaks the sleep speed again");
	check(high - low < 0.05F, "a settled wall holds within a twentieth of a stud");

	const float nominal = kStone.y * (0.5F + static_cast<float>(kRows - 1));
	const float bedding = nominal - held;

	check(bedding > 0.0F, "a wall beds into itself under its own weight");
	check(bedding < 0.02F * static_cast<float>(kRows), "a wall beds no deeper than the plastic depth per row");
}

} // namespace

void runSettling()
{
	aContactNeverReturnsMoreThanItTook();
	aRestingStackHoldsTheEngineSpacing();
	aWallSettlesAndSleeps();
}

} // namespace v8patch::tests
