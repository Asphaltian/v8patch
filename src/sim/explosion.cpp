#include "sim/simulation.h"

#include "rbx/constants.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

namespace v8patch::sim {
namespace {

constexpr float kTorqueArm = 0.5F;

[[nodiscard]] b3Vec3 getDirection(const rbx::Vector3& position, const b3Vec3& at) noexcept
{
	const b3Vec3 offset{at.x - position.x, at.y - position.y, at.z - position.z};
	const float length = b3Length(offset);

	return length > 0.0F ? b3MulSV(1.0F / length, offset) : b3Vec3{0.0F, 1.0F, 0.0F};
}

} // namespace

void Simulation::publishVelocity(Assembly& assembly)
{
	const b3Vec3 centre = b3Body_GetWorldCenter(assembly.id);
	const b3Vec3 linear = b3Body_GetLinearVelocity(assembly.id);
	const b3Vec3 rotational = b3Body_GetAngularVelocity(assembly.id);

	for (AssemblyPrimitive& part : assembly.primitives) {
		if (!part.live) {
			continue;
		}

		const b3Vec3 arm = b3Sub(toVec3(part.body->pv.position.translation), centre);

		part.body->pv.velocity.linear = fromVec3(b3Add(linear, b3Cross(rotational, arm)));
		part.body->pv.velocity.rotational = fromVec3(rotational);
		part.sentPv.velocity = part.body->pv.velocity;

		rbx::SimBody* simBody = getSimBody(part.body);

		if (simBody != nullptr) {
			simBody->dirty = 1;
		}
	}
}

void Simulation::doBlast(const Explosion& explosion, const rbx::Array<rbx::Primitive*>& found)
{
	const float reach = explosion.blastRadius + explosion.blastRadius;
	const float dt = rbx::Constants::kernelDt();

	for (rbx::Primitive* primitive : found) {
		const float radius = primitive->radius();

		if (!(radius < reach)) {
			continue;
		}

		AssemblyPrimitive* part = getAssemblyPrimitive(primitive);

		if (part == nullptr || part->assembly->bodyType != b3_dynamicBody) {
			continue;
		}

		Assembly& assembly = *part->assembly;

		const b3Vec3 at = b3Body_GetWorldPoint(assembly.id, toVec3(part->meInAssembly.translation));
		const b3Vec3 impulse =
		    b3MulSV(explosion.blastPressure * radius * radius * dt, getDirection(explosion.position, at));

		b3Body_SetAwake(assembly.id, true);
		b3Body_ApplyLinearImpulse(assembly.id, impulse, at, true);
		b3Body_ApplyAngularImpulse(assembly.id, b3MulSV(kTorqueArm * radius, impulse), true);

		publishVelocity(assembly);
	}
}

} // namespace v8patch::sim
