#include "rbx/constants.h"

#include <cmath>

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::rbx::Constants;
using v8patch::rbx::Vector3;

void stepRates()
{
	check(Constants::worldStepsPerSec() == 240, "world runs at 240 steps a second");
	check(Constants::kernelStepsPerSec() == 4560, "kernel runs at 4560 steps a second");
	check(Constants::kernelStepsPerWorldStep() == 19, "19 kernel steps to a world step");
	check(Constants::worldStepsPerUiStep() == 8, "8 world steps to a ui step");
	check(Constants::kernelStepsPerUiStep() == 152, "152 kernel steps to a ui step");

	check(
	    Constants::worldStepsPerUiStep() * Constants::kernelStepsPerWorldStep() == Constants::kernelStepsPerUiStep(),
	    "kernel steps per ui step is the product of the two rates"
	);

	near(Constants::worldDt() * Constants::worldStepsPerSec(), 1.0F, 1.0e-5F, "world dt inverts its rate");
	near(Constants::kernelDt() * Constants::kernelStepsPerSec(), 1.0F, 1.0e-5F, "kernel dt inverts its rate");
	near(Constants::uiDt() * 30.0F, 1.0F, 1.0e-5F, "ui dt is a thirtieth");
}

void elasticity()
{
	near(Constants::elasticMultiplier(0.0F), 0.28F, 1.0e-6F, "elasticity 0.00 multiplies by 0.28");
	near(Constants::elasticMultiplier(0.2F), 0.42F, 1.0e-6F, "elasticity 0.20 multiplies by 0.42");
	near(Constants::elasticMultiplier(0.5F), 0.57F, 1.0e-6F, "elasticity 0.50 multiplies by 0.57");
	near(Constants::elasticMultiplier(0.7F), 0.80F, 1.0e-6F, "elasticity 0.70 multiplies by 0.80");
	near(Constants::elasticMultiplier(1.0F), 1.00F, 1.0e-6F, "elasticity 1.00 multiplies by 1.00");

	float previous = -1.0F;

	for (int i = 0; i <= 10; ++i) {
		const float multiplier = Constants::elasticMultiplier(static_cast<float>(i) / 10.0F);
		check(multiplier >= previous, "the elasticity ladder never decreases");
		previous = multiplier;
	}
}

void jointStiffness()
{
	near(
	    Constants::jointK(Vector3{21.0F, 1.2F, 1.0F}, false),
	    21.0F * 0.4F * 960000.0F,
	    1.0F,
	    "a 21x1.2x1 plank stiffens to 8064000"
	);

	near(
	    Constants::jointK(Vector3{1.0F, 1.0F, 1.0F}, false),
	    0.91F * 960000.0F,
	    1.0F,
	    "a one stud cube stiffens to 873600"
	);

	const float thin = Constants::jointK(Vector3{0.4F, 1.0F, 1.0F}, false);
	near(thin, 0.4F * 0.91F * 960000.0F, 1.0F, "a 0.4 stud slab scales by its thinnest axis");

	check(Constants::jointK(Vector3{2.0F, 2.0F, 2.0F}, true) > 0.0F, "a ball has a positive stiffness");
}

void sleepThreshold()
{
	// sleepTolerance * weight / (1 - weight) * worldStepsPerSec
	near(
	    Constants::sleepVelocity(),
	    0.02F * 0.25F / 0.75F * 240.0F,
	    1.0e-4F,
	    "sleep velocity follows the running average"
	);
}

} // namespace

void runConstants()
{
	stepRates();
	elasticity();
	jointStiffness();
	sleepThreshold();
}

} // namespace v8patch::tests
