#include "sim/stepping.h"

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::rbx::Constants;
using v8patch::sim::Clock;
using v8patch::sim::getStepCount;

// RBX::World::step: steps = max(1, iRound(floor(worldStepsPerSec * desiredInterval))).
void stepCountFollowsTheRequestedInterval()
{
	const float rate = static_cast<float>(Constants::worldStepsPerSec());

	same(getStepCount(1.0F / 30.0F), 8, "a 30fps frame period is eight world steps");
	same(getStepCount(1.0F / 60.0F), 4, "a 60fps frame period is four");
	same(getStepCount(1.0F), Constants::worldStepsPerSec(), "a whole second is the full rate");
	same(getStepCount(2.0F), 2 * Constants::worldStepsPerSec(), "and two seconds is twice that");
	same(getStepCount(3.9F / rate), 3, "the remainder is dropped, not rounded up");
}

// RunService leaves framePeriod at zero when the frame rate is uncapped, and the
// engine still advances one step rather than none.
void anIntervalTooShortForAStepStillTakesOne()
{
	same(getStepCount(0.0F), 1, "an uncapped frame period is one step");
	same(getStepCount(1.0F / 4560.0F), 1, "and so is a kernel step's worth");
	same(getStepCount(-1.0F), 1, "a negative interval cannot ask for fewer");
}

// The engine never carries unspent time forward. A frame that overruns dilates
// simulated time instead, which is why 11,520 parts advance 0.75s of simulation
// in 56s of wall clock. Catching up here instead ran the world 75x faster than
// the engine and collapsed every stack in the arena.
void timeIsNotCarriedBetweenFrames()
{
	const int period = getStepCount(1.0F / 30.0F);

	for (int i = 0; i < 16; ++i) {
		same(getStepCount(1.0F / 30.0F), period, "an unchanging interval asks for an unchanging step count");
	}

	same(
	    getStepCount(1.0F / 30.0F) + getStepCount(1.0F / 30.0F),
	    getStepCount(2.0F / 30.0F),
	    "two frames advance exactly what one of twice the interval does"
	);
}

// Uncapping the frame rate leaves framePeriod at zero, so the interval comes from
// the wall clock instead. It is still capped at what RunService would have asked
// for: a frame that overran must not hand the world the whole overrun at once.
void anUncappedFramePeriodPacesAgainstTheClock()
{
	Clock clock;

	same(clock.take(0.0F, 1.0F / 30.0F), 8, "a 30fps frame is still eight steps");
	same(clock.take(0.0F, 1.0F / 240.0F), 1, "and a step's worth is one");
	same(clock.take(0.0F, 0.0F), 0, "no elapsed time asks for no steps");
}

// A frame that overran dilates simulated time, exactly as the engine does when
// RunService cannot hold its period. Advancing the whole overrun instead ran the
// arena seven times fast and pulled all five towers down.
void anOverrunIsDilatedNotCaughtUp()
{
	Clock clock;

	const int budget = static_cast<int>(Clock::budget() * static_cast<float>(Constants::worldStepsPerSec()));

	same(budget, Constants::worldStepsPerUiStep(), "the ceiling is one UI step of simulation");
	same(clock.take(0.0F, 1.0F), budget, "a one second stall advances the ceiling, not a second");
	same(clock.take(0.0F, 10.0F), budget, "and neither does a ten second one");
	check(clock.pending() < Constants::worldDt(), "nothing is left owing afterwards");
}

// Sub-step intervals accumulate rather than being dropped, or an uncapped client
// would run the world slower the faster it rendered.
void shortFramesAccumulate()
{
	Clock clock;

	int steps = 0;

	for (int i = 0; i < 240; ++i) {
		steps += clock.take(0.0F, 1.0F / 960.0F);
	}

	same(steps, 60, "four frames of a quarter step each make one step");
}

} // namespace

void runStepping()
{
	stepCountFollowsTheRequestedInterval();
	anIntervalTooShortForAStepStillTakesOne();
	timeIsNotCarriedBetweenFrames();
	anUncappedFramePeriodPacesAgainstTheClock();
	anOverrunIsDilatedNotCaughtUp();
	shortFramesAccumulate();
}

} // namespace v8patch::tests
