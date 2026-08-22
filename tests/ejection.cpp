#include <cmath>

#include "check.h"
#include "world.h"

namespace v8patch::tests {
namespace {

using v8patch::rbx::Vector3;

Vector3 cube(float side)
{
	return Vector3{side, side, side};
}

void overlappingCubesEjectAtTheEngineRate()
{
	const Bed bed;

	bed.part(cube(30.0F), b3Vec3{0.0F, -15.0F, 0.0F}, true);

	b3BodyId made[6]{};

	for (int i = 0; i < 6; ++i) {
		made[i] = bed.part(cube(4.0F), b3Vec3{0.0F, 2.0F + 0.5F * static_cast<float>(i), 0.0F}, false);
	}

	float peak = 0.0F;

	for (int beat = 0; beat < 240; ++beat) {
		bed.step();

		for (const b3BodyId id : made) {
			const b3Vec3 v = b3Body_GetLinearVelocity(id);
			const float speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

			if (speed > peak) {
				peak = speed;
			}
		}
	}

	check(peak > 40.0F, "six cubes overlapping by 3.5 studs eject at the engine's rate");
	check(peak < 400.0F, "the ejection does not fling them at the speed cap");
}

void aRestingContactHoldsTheEngineOverlap()
{
	const Bed bed;

	bed.part(cube(30.0F), b3Vec3{0.0F, -15.0F, 0.0F}, true);

	const b3BodyId brick = bed.part(cube(4.0F), b3Vec3{0.0F, 2.0F, 0.0F}, false);

	for (int beat = 0; beat < 480; ++beat) {
		bed.step();
	}

	const b3Pos at = b3Body_GetPosition(brick);
	const float sank = 2.0F - at.y;

	check(sank > 0.004F, "a resting contact settles into the surface");
	check(sank < 0.040F, "a resting contact does not sink past the engine's overlap");
}

void aFastPartIsNotStoppedDeadByOneContact()
{
	const Bed bed;

	bed.part(cube(8.0F), b3Vec3{0.0F, 0.0F, 0.0F}, true);

	const b3BodyId shot = bed.part(cube(2.0F), b3Vec3{-20.0F, 0.0F, 0.0F}, false);
	b3Body_SetGravityScale(shot, 0.0F);
	b3Body_SetLinearVelocity(shot, b3Vec3{300.0F, 0.0F, 0.0F});

	bed.step();
	bed.step();

	const b3Vec3 v = b3Body_GetLinearVelocity(shot);

	check(v.x > 200.0F, "a contact cannot take a fast part's whole velocity in one step");
}

void aFlushNeighbourCarriesNoLoad()
{
	const Bed bed;

	bed.part(cube(4.0F), b3Vec3{4.0F, 0.0F, 0.0F}, true);

	const b3BodyId slider = bed.part(cube(4.0F), b3Vec3{0.0F, 0.0F, 0.0F}, false);
	b3Body_SetGravityScale(slider, 0.0F);
	b3Body_SetLinearVelocity(slider, b3Vec3{0.0F, 0.0F, 40.0F});

	for (int beat = 0; beat < 48; ++beat) {
		bed.step();
	}

	const b3Vec3 v = b3Body_GetLinearVelocity(slider);

	check(v.z > 39.0F, "a face set flush neither grips a part nor brakes it");
	check(std::fabs(v.x) < 1.0F, "and does not push it off the face either");
}

void aFastPartPassesThroughAThinWall()
{
	const Bed bed;

	bed.part(Vector3{1.0F, 20.0F, 20.0F}, b3Vec3{0.0F, 0.0F, 0.0F}, true);

	const b3BodyId shot = bed.part(cube(1.0F), b3Vec3{-24.0F, 0.0F, 0.0F}, false);
	b3Body_SetGravityScale(shot, 0.0F);
	b3Body_SetLinearVelocity(shot, b3Vec3{300.0F, 0.0F, 0.0F});

	for (int beat = 0; beat < 60; ++beat) {
		bed.step();
	}

	const b3Pos at = b3Body_GetPosition(shot);

	check(at.x > 1.0F, "a part at 300 studs per second passes through a one stud wall, as the engine does");
}

} // namespace

void runEjection()
{
	overlappingCubesEjectAtTheEngineRate();
	aRestingContactHoldsTheEngineOverlap();
	aFastPartIsNotStoppedDeadByOneContact();
	aFlushNeighbourCarriesNoLoad();
	aFastPartPassesThroughAThinWall();
}

} // namespace v8patch::tests
