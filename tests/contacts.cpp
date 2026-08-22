#include "rbx/engine.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

#include "check.h"

namespace v8patch::tests {
namespace {

using v8patch::rbx::Vector3;
using v8patch::sim::makeHull;

int collide(const Vector3& sizeA, const Vector3& sizeB, const b3Vec3& offset, b3SATCache& cache, float keepDistance)
{
	const b3Transform origin{b3Vec3_zero, b3Quat_identity};

	alignas(16) const b3BoxHull a = makeHull(sizeA, origin);
	alignas(16) const b3BoxHull b = makeHull(sizeB, origin);

	b3LocalManifoldPoint points[B3_MAX_MANIFOLD_POINTS]{};
	b3LocalManifold manifold{};

	manifold.points = points;

	b3CollideHulls(
	    &manifold, B3_MAX_MANIFOLD_POINTS, &a.base, &b.base, b3Transform{offset, b3Quat_identity}, &cache, keepDistance
	);

	return manifold.pointCount;
}

float trackedDistance()
{
	return B3_SPECULATIVE_DISTANCE;
}

int collide(const Vector3& sizeA, const Vector3& sizeB, const b3Vec3& offset)
{
	b3SATCache cache{};

	return collide(sizeA, sizeB, offset, cache, trackedDistance());
}

int collide(const Vector3& sizeA, const Vector3& sizeB, const b3Vec3& offset, float keepDistance)
{
	b3SATCache cache{};

	return collide(sizeA, sizeB, offset, cache, keepDistance);
}

void separatedShapesMakeNoContact()
{
	const Vector3 cube{2.0F, 2.0F, 2.0F};

	check(collide(cube, cube, b3Vec3{0.0F, 1.9F, 0.0F}) > 0, "boxes overlapping by 0.1 studs are tracked");
	check(collide(cube, cube, b3Vec3{0.0F, 2.0F, 0.0F}) > 0, "boxes exactly flush are tracked");
	check(collide(cube, cube, b3Vec3{0.0F, 2.3F, 0.0F}) > 0, "boxes inside the speculative margin are tracked");
	check(collide(cube, cube, b3Vec3{0.0F, 2.5F, 0.0F}) == 0, "boxes past the margin are not tracked");
	check(collide(cube, cube, b3Vec3{0.0F, 6.0F, 0.0F}) == 0, "boxes far apart are not tracked");
}

void aPairIsTrackedBeforeItTouches()
{
	const Vector3 plank{21.0F, 1.2F, 1.0F};
	const Vector3 post{2.0F, 12.0F, 2.0F};

	check(collide(plank, post, b3Vec3{5.5F, -6.59F, 1.49F}) > 0, "a plank overlapping a post is tracked");
	check(collide(plank, post, b3Vec3{5.5F, -6.2F, 1.8F}) > 0, "and so is one still approaching it");
	check(collide(plank, post, b3Vec3{5.5F, -6.2F, 2.6F}) == 0, "a plank well clear of a post is not");
}

void runningBondCoursesTouch()
{
	const Vector3 brick{2.0F, 1.2F, 4.0F};

	check(collide(brick, brick, b3Vec3{1.0F, 1.2F, 0.0F}) > 0, "a course laid in a running bond rests on the one below");
	check(
	    collide(brick, brick, b3Vec3{1.0F, 1.19F, 0.0F}) > 0,
	    "and still touches once its own weight has bedded it in"
	);
	check(collide(brick, brick, b3Vec3{1.0F, 1.0F, 0.0F}) > 0, "and touches while deeply overlapped");
	check(collide(brick, brick, b3Vec3{1.0F, 1.7F, 0.0F}) == 0, "but not when it is lifted clear");
}

void flushNeighboursAreTracked()
{
	const Vector3 brick{2.0F, 1.2F, 4.0F};

	check(collide(brick, brick, b3Vec3{2.0F, 0.0F, 0.0F}) > 0, "a course neighbour set flush is tracked");
	check(collide(brick, brick, b3Vec3{2.0005F, 0.0F, 0.0F}) > 0, "and so is one the grid rounded a hair clear");
	check(collide(brick, brick, b3Vec3{2.1F, 0.0F, 0.0F}) > 0, "and one a tenth of a stud clear");
	check(collide(brick, brick, b3Vec3{2.5F, 0.0F, 0.0F}) == 0, "but not one half a stud clear");
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

void contactsDoNotOutlastTheSurface()
{
	const Vector3 cube{2.0F, 2.0F, 2.0F};

	b3SATCache cache{};

	check(
	    collide(cube, cube, b3Vec3{0.0F, 1.99F, 0.0F}, cache, trackedDistance()) > 0,
	    "a loaded course overlaps and touches"
	);
	check(
	    collide(cube, cube, b3Vec3{0.0F, 2.6F, 0.0F}, cache, trackedDistance()) == 0,
	    "and lets go the moment it lifts clear"
	);
	check(
	    collide(cube, cube, b3Vec3{0.0F, 1.99F, 0.0F}, cache, trackedDistance()) > 0,
	    "and takes it back on the way down"
	);
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
	aPairIsTrackedBeforeItTouches();
	flushStackHoldsContacts();
	runningBondCoursesTouch();
	flushNeighboursAreTracked();
	contactsDoNotOutlastTheSurface();
	deepOverlapStillContacts();
}

} // namespace v8patch::tests
