#include "rbx/engine.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::rbx::Vector3;
using v8patch::sim::makeHull;

int collide(const Vector3& sizeA, const Vector3& sizeB, const b3Vec3& offset)
{
	const b3Transform origin{b3Vec3_zero, b3Quat_identity};

	alignas(16) const b3BoxHull a = makeHull(sizeA, origin);
	alignas(16) const b3BoxHull b = makeHull(sizeB, origin);

	b3LocalManifoldPoint points[B3_MAX_MANIFOLD_POINTS]{};
	b3LocalManifold manifold{};
	b3SATCache cache{};

	manifold.points = points;

	b3CollideHulls(&manifold, B3_MAX_MANIFOLD_POINTS, &a.base, &b.base, b3Transform{offset, b3Quat_identity}, &cache);

	return manifold.pointCount;
}

void separatedShapesMakeNoContact()
{
	const Vector3 cube{2.0F, 2.0F, 2.0F};

	check(collide(cube, cube, b3Vec3{0.0F, 1.9F, 0.0F}) > 0, "boxes overlapping by 0.1 studs touch");
	check(collide(cube, cube, b3Vec3{0.0F, 2.0F, 0.0F}) > 0, "boxes exactly flush still touch");
	check(collide(cube, cube, b3Vec3{0.0F, 2.1F, 0.0F}) == 0, "boxes 0.1 studs apart do not touch");
	check(collide(cube, cube, b3Vec3{0.0F, 2.3F, 0.0F}) == 0, "boxes inside the speculative margin still do not touch");
	check(collide(cube, cube, b3Vec3{0.0F, 6.0F, 0.0F}) == 0, "boxes far apart do not touch");
}

void flushFacesMakeNoContact()
{
	const Vector3 plank{21.0F, 1.2F, 1.0F};
	const Vector3 post{2.0F, 12.0F, 2.0F};

	check(collide(plank, post, b3Vec3{5.5F, -6.2F, 1.9F}) == 0, "a plank clear of a post makes no contact");

	check(
	    collide(plank, post, b3Vec3{5.5F, -6.59F, 1.49F}) > 0,
	    "the same pair overlapping by 0.01 studs does make contact"
	);
}

void flushStackHoldsContacts()
{
	const Vector3 brick{4.0F, 2.0F, 2.0F};

	for (int course = 2; course <= 8; ++course) {
		const float lift = 2.0F * static_cast<float>(course);

		check(collide(brick, brick, b3Vec3{0.0F, lift, 0.0F}) == 0, "courses that do not touch make no contact");
	}

	check(collide(brick, brick, b3Vec3{0.0F, 2.0F, 0.0F}) > 0, "the course directly above rests in contact");
	check(collide(brick, brick, b3Vec3{2.0F, 2.0F, 0.0F}) > 0, "and so does one offset in a running bond");
}

void deepOverlapStillContacts()
{
	const Vector3 cube{2.0F, 2.0F, 2.0F};

	check(collide(cube, cube, b3Vec3{0.0F, 1.0F, 0.0F}) > 0, "a half buried box touches");
	check(collide(cube, cube, b3Vec3{0.0F, 0.1F, 0.0F}) > 0, "an almost coincident box touches");
}

int reportAssert(const char* condition, const char* file, int line)
{
	std::printf("box3d assert: %s at %s:%d\n", condition, file, line);

	return 0;
}

} // namespace

void runContacts()
{
	b3SetAssertFcn(&reportAssert);
	b3SetLengthUnitsPerMeter(v8patch::rbx::Units::studsPerMetre);

	separatedShapesMakeNoContact();
	flushFacesMakeNoContact();
	flushStackHoldsContacts();
	deepOverlapStillContacts();
}

} // namespace v8patch::tests
