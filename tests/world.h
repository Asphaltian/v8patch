#pragma once

#include <box3d/box3d.h>

#include "rbx/constants.h"
#include "sim/contacts.h"
#include "sim/primitives.h"

namespace v8patch::tests {

struct Bed
{
	b3WorldId world{b3_nullWorldId};

	explicit Bed(bool sleeps = false)
	{
		b3SetLengthUnitsPerMeter(rbx::Units::studsPerMetre);

		const float substepDt = rbx::Constants::worldDt() / static_cast<float>(sim::kSubSteps);
		const rbx::Vector3 gravity = rbx::Units::kmsAccelerationToRbx(rbx::Constants::kmsGravity());

		b3WorldDef def = b3DefaultWorldDef();
		def.gravity = b3Vec3{gravity.x, gravity.y, gravity.z};
		def.contactHertz = sim::getStiffestHertz(substepDt);
		def.contactDampingRatio = sim::getContactDamping(def.contactHertz, substepDt);
		def.frictionCallback = &sim::getFriction;
		def.restitutionCallback = &sim::getRestitution;
		def.enableSleep = sleeps;
		def.workerCount = 1;

		world = b3CreateWorld(&def);
	}

	~Bed() { b3DestroyWorld(world); }

	Bed(const Bed&) = delete;
	Bed& operator=(const Bed&) = delete;

	void step() const { b3World_Step(world, rbx::Constants::worldDt(), sim::kSubSteps); }

	b3BodyId part(rbx::Vector3 size, b3Vec3 at, bool anchored, float elasticity = 0.0F, float friction = 0.3F) const
	{
		b3BodyDef body = b3DefaultBodyDef();
		body.type = anchored ? b3_staticBody : b3_dynamicBody;
		body.position = b3Pos{at.x, at.y, at.z};
		body.sleepThreshold = sim::kSleepVelocity;

		const b3BodyId id = b3CreateBody(world, &body);

		b3ShapeDef shape = b3DefaultShapeDef();
		shape.baseMaterial.friction = friction;
		shape.baseMaterial.restitution = elasticity;
		shape.baseMaterial.jointK = rbx::Constants::jointK(size, false);
		shape.density = 1.0F;

		const b3Transform origin{b3Vec3_zero, b3Quat_identity};
		alignas(16) const b3BoxHull hull = sim::makeHull(size, origin);
		b3CreateHullShape(id, &shape, &hull.base);

		return id;
	}

	int settle(b3BodyId watched, int limit) const
	{
		for (int beat = 0; beat < limit; ++beat) {
			step();

			if (!b3Body_IsAwake(watched)) {
				return beat;
			}
		}

		return -1;
	}
};

} // namespace v8patch::tests
