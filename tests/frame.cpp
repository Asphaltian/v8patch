#include "frame.h"

#include <chrono>
#include <cstdlib>

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::Frame;
using v8patch::KeyHold;
using v8patch::Pacer;
using v8patch::UiStep;
using v8patch::rbx::Constants;

int raises(UiStep& step, float frame, int frames)
{
	int raised = 0;

	for (int i = 0; i < frames; ++i) {
		if (step.ready(frame, frame)) {
			++raised;
		}
	}

	return raised;
}

void aFastFrameRateStillRaisesThirtyTimesASecond()
{
	UiStep fast;
	UiStep quick;
	UiStep native;

	same(raises(fast, 1.0F / 500.0F, 500), 30, "five hundred frames raise thirty");
	same(raises(quick, 1.0F / 240.0F, 240), 30, "and so do two hundred and forty");
	same(raises(native, 1.0F / 60.0F, 60), 30, "and sixty");
}

void aSlowFrameRateIsNeverHeldBack()
{
	UiStep slow;
	UiStep native;

	same(raises(slow, 1.0F / 20.0F, 20), 20, "twenty frames raise twenty");
	same(raises(native, 1.0F / 30.0F, 30), 30, "thirty raise thirty");
}

void theWithheldStepArrivesWhole()
{
	UiStep step;

	float delivered = 0.0F;

	for (int i = 0; i < 500; ++i) {
		if (step.ready(1.0F / 500.0F, 1.0F / 500.0F)) {
			delivered += step.delivered();
		}
	}

	near(delivered + step.pending(), 1.0F, 1e-4F, "a second of frames hands over a second of step");
}

void aZeroStepStillRaises()
{
	UiStep idle;

	int raised = 0;

	for (int i = 0; i < 500; ++i) {
		if (idle.ready(1.0F / 500.0F, 0.0F)) {
			++raised;
		}
	}

	same(raised, 30, "a second of zero-step frames still raises thirty");
}

void aStallIsNotPaidBack()
{
	UiStep step;

	check(step.ready(3.0F, 3.0F), "the frame after a three second stall raises");
	same(raises(step, 1.0F / 500.0F, 500), 31, "and the second after owes one frame, not ninety");
}

void whatIsDeliveredIsWhatAccumulated()
{
	UiStep step;

	check(!step.ready(0.01F, 0.01F), "a third of a frame is not a frame");
	check(!step.ready(0.01F, 0.01F), "nor is two thirds");
	check(step.ready(0.02F, 0.02F), "the fourth crosses");

	near(step.delivered(), 0.04F, 1e-6F, "and hands over all four hundredths");
	near(step.pending(), 0.0F, 1e-6F, "with nothing left owing");
}

float travelled(float frame, int frames)
{
	KeyHold held;

	float studs = 0.0F;

	for (int i = 0; i < frames; ++i) {
		studs += 2.0F * held.advance(frame);
	}

	return studs;
}

void theFreeCameraCoversTheSameGroundAtAnyFrameRate()
{
	near(travelled(1.0F / 500.0F, 500), 60.0F, 0.5F, "five hundred frames cover sixty studs");
	near(travelled(1.0F / 240.0F, 240), 60.0F, 0.5F, "and so do two hundred and forty");
	near(travelled(1.0F / 60.0F, 60), 60.0F, 0.5F, "and sixty");
	near(travelled(1.0F / 30.0F, 30), 60.0F, 0.5F, "and the thirty the engine ran at");
}

void aSlowFrameNeverTakesMoreThanAWholeMove()
{
	KeyHold held;

	near(travelled(1.0F / 15.0F, 15), 30.0F, 0.5F, "fifteen frames cover half the ground");
	near(held.advance(3.0F), 1.0F, 1e-6F, "a three second stall is still one move");
}

float secondsUntilCalls(float frame, int wanted)
{
	KeyHold held;

	float seconds = 0.0F;

	while (held.calls() < wanted) {
		(void)held.advance(frame);
		seconds += frame;
	}

	return seconds;
}

void theSpeedRampIsCountedInTime()
{
	KeyHold held;

	const float faster = 61.0F * Constants::uiDt();
	const float fastest = 121.0F * Constants::uiDt();

	same(held.calls(), 0, "an untouched key has taken no calls");

	near(secondsUntilCalls(1.0F / 500.0F, 61), faster, 0.01F, "four studs a call two seconds in at five hundred");
	near(secondsUntilCalls(1.0F / 60.0F, 61), faster, 0.02F, "and the same two seconds in at sixty");
	near(secondsUntilCalls(1.0F / 500.0F, 121), fastest, 0.01F, "and eight studs a call four seconds in");

	(void)held.advance(1.0F);
	held.release();

	same(held.calls(), 0, "and letting go starts again");
}

int opensIn(Pacer& pacer, std::chrono::milliseconds window, bool ownStep)
{
	const auto deadline = std::chrono::steady_clock::now() + window;

	int opened = 0;

	while (std::chrono::steady_clock::now() < deadline) {
		if (ownStep ? pacer.ready() : pacer.ready(0.0F)) {
			++opened;
		}
	}

	return opened;
}

void aPacerOpensThirtyTimesASecond()
{
	using namespace std::chrono_literals;

	Pacer paced;
	Pacer stepless;

	const int wanted = 15;

	check(std::abs(opensIn(paced, 500ms, false) - wanted) <= 2, "half a second of frames opens fifteen times");
	check(std::abs(opensIn(stepless, 500ms, true) - wanted) <= 2, "and so does one measuring its own step");
}

void aFrameIsConsumedByReadingIt()
{
	Frame frame;

	float first = 0.0F;

	while (first < 0.001F) {
		first = frame.elapsed();
	}

	check(frame.elapsed() < first / 4.0F, "reading a frame twice leaves nothing for the second read");
}

} // namespace

void runFrame()
{
	aFastFrameRateStillRaisesThirtyTimesASecond();
	aSlowFrameRateIsNeverHeldBack();
	theWithheldStepArrivesWhole();
	aZeroStepStillRaises();
	aStallIsNotPaidBack();
	whatIsDeliveredIsWhatAccumulated();
	theFreeCameraCoversTheSameGroundAtAnyFrameRate();
	aSlowFrameNeverTakesMoreThanAWholeMove();
	theSpeedRampIsCountedInTime();
	aPacerOpensThirtyTimesASecond();
	aFrameIsConsumedByReadingIt();
}

} // namespace v8patch::tests
