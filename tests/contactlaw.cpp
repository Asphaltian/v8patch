#include "sim/contacts.h"

#include <cmath>

#include "check.h"

namespace v8patch::tests {
namespace {

using namespace v8patch::sim;
using v8patch::rbx::Constants;

void frictionTakesTheDullerSurface()
{
	near(getFriction(0.3F, 0, 0.9F, 0), 0.3F, 1.0e-6F, "friction takes the lower of the pair");
	near(getFriction(0.9F, 0, 0.3F, 0), 0.3F, 1.0e-6F, "friction does not care about order");
	near(getFriction(0.0F, 0, 0.9F, 0), 0.0F, 1.0e-6F, "a frictionless surface wins");
}

// Measured against the 2007 engine: a brick dropped 57.6 studs rebounds with
// these coefficients, so the law must stay sqrt of the elasticity multiplier.
void restitutionFollowsTheLadder()
{
	near(getRestitution(0.0F, 0, 0.0F, 0), std::sqrt(0.28F), 1.0e-6F, "elasticity 0.00 bounces at sqrt 0.28");
	near(getRestitution(0.2F, 0, 0.2F, 0), std::sqrt(0.42F), 1.0e-6F, "elasticity 0.20 bounces at sqrt 0.42");
	near(getRestitution(0.5F, 0, 0.5F, 0), std::sqrt(0.57F), 1.0e-6F, "elasticity 0.50 bounces at sqrt 0.57");
	near(getRestitution(0.7F, 0, 0.7F, 0), std::sqrt(0.80F), 1.0e-6F, "elasticity 0.70 bounces at sqrt 0.80");
	near(getRestitution(1.0F, 0, 1.0F, 0), 1.0F, 1.0e-6F, "elasticity 1.00 bounces at one");

	near(
	    getRestitution(1.0F, 0, 0.0F, 0), std::sqrt(0.28F), 1.0e-6F, "a dead surface kills the bounce of a lively one"
	);

	for (int i = 0; i <= 10; ++i) {
		const float e = getRestitution(static_cast<float>(i) / 10.0F, 0, 1.0F, 0);
		check(e >= 0.0F && e <= 1.0F, "restitution never leaves the unit range");
	}
}

// Restitution above one creates energy: a brick would climb higher than it fell.
void restitutionNeverCreatesEnergy()
{
	for (int i = 0; i <= 20; ++i) {
		const float elasticity = static_cast<float>(i) / 20.0F;
		check(getRestitution(elasticity, 0, elasticity, 0) <= 1.0F, "restitution stays at or below one");
	}
}

// An eighth of the step rate, 30Hz at 240. b3SolverStage clamps contactHertz to
// 0.125 / h and asking for more is not merely ignored: getContactDamping derives
// its damping from the hertz we ask for, so a request the solver clamps away
// leaves the pair mismatched and halves the recovery rate.
void stiffnessMatchesTheStepRate()
{
	const float substepDt = Constants::worldDt();
	const float rate = static_cast<float>(Constants::worldStepsPerSec());

	near(getStiffestHertz(substepDt), 0.125F * rate, 1.0e-2F, "the contact spring is an eighth of the step rate");
	near(getStiffestHertz(substepDt), 30.0F, 1.0e-2F, "which is 30Hz at 240 steps a second");
	check(getStiffestHertz(substepDt) <= 0.125F * rate, "and does not exceed the solver clamp");
}

// getContactDamping is the inverse of getBiasRate at the engine's own recovery
// rate, so feeding one into the other must come back where it started.
void dampingInvertsTheBiasRate()
{
	const float substepDt = Constants::worldDt();
	const float hertz = getStiffestHertz(substepDt);
	const float damping = getContactDamping(hertz, substepDt);

	check(damping > 0.0F, "the derived damping is positive");
	near(
	    getBiasRate(hertz, damping, substepDt),
	    Constants::contactBiasRate(),
	    1.0e-2F,
	    "the derived damping reproduces the engine recovery rate"
	);
}

} // namespace

void runContactlaw()
{
	frictionTakesTheDullerSurface();
	restitutionFollowsTheLadder();
	restitutionNeverCreatesEnergy();
	stiffnessMatchesTheStepRate();
	dampingInvertsTheBiasRate();
}

} // namespace v8patch::tests
