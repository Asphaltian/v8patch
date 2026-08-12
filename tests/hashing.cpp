#include "rbx/engine.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

#include <cmath>
#include <set>

#include "check.h"

namespace v8patch::tests {
namespace {

using namespace v8patch::sim;
using v8patch::rbx::CoordinateFrame;
using v8patch::rbx::Geometry;
using v8patch::rbx::Primitive;
using v8patch::rbx::Vector3;

// The rebuild check must notice a primitive whose address is reused by a
// different part in the same frame, which a commutative pointer sum cannot.
void signatureOrdering()
{
	int a = 0;
	int b = 0;
	int c = 0;

	const auto* first = reinterpret_cast<const Primitive*>(&a);
	const auto* second = reinterpret_cast<const Primitive*>(&b);
	const auto* third = reinterpret_cast<const Primitive*>(&c);

	const auto run = [](const Primitive* one, const Primitive* two, const Primitive* three) {
		std::uint64_t seed = 0;
		seed = signatureOf(one, seed);
		seed = signatureOf(two, seed);

		return signatureOf(three, seed);
	};

	check(run(first, second, third) == run(first, second, third), "the signature is stable for one order");
	check(run(first, second, third) != run(third, second, first), "the signature sees a permutation");
	check(run(first, second, third) != run(first, second, second), "the signature sees a swapped member");
}

void signatureSpreads()
{
	std::set<std::uint64_t> seen;
	alignas(16) char block[64]{};

	for (int i = 0; i < 32; ++i) {
		seen.insert(signatureOf(reinterpret_cast<const Primitive*>(block + i), 0));
	}

	check(seen.size() == 32, "neighbouring addresses hash apart");
}

// clumpHashOf folds in mass and geometry so an assembly built before the
// engine filled those in is rebuilt once they arrive.
void clumpHashTracksMass()
{
	v8patch::rbx::Body body{};
	Geometry geometry{};
	Primitive primitive{};

	geometry.gridSize = Vector3{4.0F, 4.0F, 4.0F};
	body.mass = 0.0F;
	primitive.body = &body;
	primitive.geometry = &geometry;

	const std::uint64_t massless = clumpHashOf(&primitive);

	body.mass = 64.0F;
	const std::uint64_t weighed = clumpHashOf(&primitive);

	check(massless != weighed, "the clump hash notices mass arriving");

	geometry.gridSize = Vector3{2.0F, 2.0F, 2.0F};
	check(clumpHashOf(&primitive) != weighed, "the clump hash notices a resize");

	primitive.geometry = nullptr;
	check(clumpHashOf(&primitive) != weighed, "the clump hash notices a missing geometry");
}

void apartThresholds()
{
	const Vector3 origin{0.0F, 0.0F, 0.0F};

	check(!apart(origin, Vector3{0.0F, 5.0e-4F, 0.0F}, 1.0e-3F), "half a millistud is not apart");
	check(apart(origin, Vector3{0.0F, 2.0e-3F, 0.0F}, 1.0e-3F), "two millistuds is apart");

	CoordinateFrame here{};
	CoordinateFrame there{};
	there.translation = Vector3{1.0F, 0.0F, 0.0F};

	check(!apart(here, here), "a frame is not apart from itself");
	check(apart(here, there), "a stud of travel is apart");
}

void hullMatchesGrid()
{
	const b3Transform at{b3Vec3_zero, b3Quat_identity};
	const b3BoxHull hull = makeHull(Vector3{4.0F, 2.0F, 6.0F}, at);

	const b3Vec3 lower = hull.base.aabb.lowerBound;
	const b3Vec3 upper = hull.base.aabb.upperBound;

	check(std::fabs(upper.x - lower.x - 4.0F) < 1.0e-5F, "the hull keeps its width");
	check(std::fabs(upper.y - lower.y - 2.0F) < 1.0e-5F, "the hull keeps its height");
	check(std::fabs(upper.z - lower.z - 6.0F) < 1.0e-5F, "the hull keeps its depth");
}

void jointKinds()
{
	check(isRigidJoint(v8patch::rbx::Joint::Weld), "a weld is rigid");
	check(isRigidJoint(v8patch::rbx::Joint::Snap), "a snap is rigid");
	check(isRigidJoint(v8patch::rbx::Joint::Motor), "a motor is rigid");
	check(!isRigidJoint(v8patch::rbx::Joint::Rotate), "a rotate is not rigid");

	check(isConnectorJoint(v8patch::rbx::Joint::Rotate), "a rotate drives a connector");
	check(isConnectorJoint(v8patch::rbx::Joint::RotateP), "a rotate p drives a connector");
	check(isConnectorJoint(v8patch::rbx::Joint::RotateV), "a rotate v drives a connector");
	check(isConnectorJoint(v8patch::rbx::Joint::Glue), "a glue drives a connector");
	check(!isConnectorJoint(v8patch::rbx::Joint::Weld), "a weld drives no connector");
}

} // namespace

void runHashing()
{
	signatureOrdering();
	signatureSpreads();
	clumpHashTracksMass();
	apartThresholds();
	hullMatchesGrid();
	jointKinds();
}

} // namespace v8patch::tests
