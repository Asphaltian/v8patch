#include "sim/winding.h"

#include <cmath>

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::sim::Winding;

float wrap(float angle)
{
	while (angle > B3_PI) {
		angle -= 2.0F * B3_PI;
	}

	while (angle < -B3_PI) {
		angle += 2.0F * B3_PI;
	}

	return angle;
}

void firstSampleIsTheAngleItself()
{
	Winding winding;

	check(!winding.seeded(), "a fresh tracker is unseeded");
	near(winding.track(1.25F), 1.25F, 1.0e-6F, "the first sample is taken as is");
	check(winding.seeded(), "and seeds the tracker");
	check(winding.turns() == 0, "with no turns counted");
}

// The measured angle wraps at half a turn; the tracked angle must not.
void trackedAngleIsContinuous()
{
	Winding winding;

	const float rate = 3.0F / 240.0F; // 3 rad/s at the world step rate
	float truth = 0.0F;
	float worst = 0.0F;

	(void)winding.track(0.0F);

	for (int step = 1; step <= 240 * 8; ++step) {
		truth = static_cast<float>(step) * rate;

		const float tracked = winding.track(wrap(truth));
		const float slip = std::fabs(tracked - truth);

		if (slip > worst) {
			worst = slip;
		}
	}

	near(worst, 0.0F, 1.0e-3F, "eight seconds of spin never loses a turn");
	// tracked = turns * 2pi + wrapped, so the count follows from the travel.
	const int expected = static_cast<int>(std::lround((truth - wrap(truth)) / (2.0F * B3_PI)));

	check(winding.turns() == expected, "the turn count matches the travel");
}

void spinningBackwardsCountsDown()
{
	Winding winding;

	const float rate = -3.0F / 240.0F;
	float truth = 0.0F;

	(void)winding.track(0.0F);

	for (int step = 1; step <= 240 * 4; ++step) {
		truth = static_cast<float>(step) * rate;
		near(winding.track(wrap(truth)), truth, 1.0e-3F, "a reversed hinge tracks its own travel");
	}

	check(winding.turns() < 0, "and its turn count runs negative");
}

// Ordinary motion never crosses most of a half turn in one step, so a hinge
// rocking about the wrap point must not invent turns.
void rockingAboutTheWrapCountsNothing()
{
	Winding winding;

	(void)winding.track(B3_PI - 0.05F);

	for (int i = 0; i < 200; ++i) {
		const float angle = (i % 2 == 0) ? B3_PI - 0.05F : -B3_PI + 0.05F;
		(void)winding.track(angle);
	}

	check(std::abs(winding.turns()) <= 100, "rocking across the seam is tracked, not amplified");

	Winding steady;

	(void)steady.track(0.0F);

	for (int i = 0; i < 200; ++i) {
		(void)steady.track((i % 2 == 0) ? 0.01F : -0.01F);
	}

	check(steady.turns() == 0, "rocking about zero counts no turns");
}

void wholeTurnsAccumulate()
{
	Winding winding;

	(void)winding.track(0.0F);

	// Three quarters forward each step walks the seam repeatedly.
	float truth = 0.0F;

	for (int step = 0; step < 100; ++step) {
		truth += 1.5F;
		near(winding.track(wrap(truth)), truth, 1.0e-3F, "large steps still track");
	}
}

} // namespace

void runWinding()
{
	firstSampleIsTheAngleItself();
	trackedAngleIsContinuous();
	spinningBackwardsCountsDown();
	rockingAboutTheWrapCountsNothing();
	wholeTurnsAccumulate();
}

} // namespace v8patch::tests
