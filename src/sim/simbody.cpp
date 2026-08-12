#include "sim/simulation.h"

#include <vector>

#include "rbx/constants.h"
#include "rbx/dynamics.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

namespace v8patch::sim {

void Simulation::updateBodies()
{
	for (auto& held : assemblies_) {
		Assembly& assembly = *held;

		if (assembly.bodyType == b3_staticBody) {
			continue;
		}

		AssemblyPrimitive* moved = nullptr;
		bool coordChanged = false;
		bool velocityChanged = false;

		for (AssemblyPrimitive& part : assembly.primitives) {
			if (!part.live || part.body->stateIndex == part.stateIndex) {
				continue;
			}

			part.stateIndex = part.body->stateIndex;

			const bool partMoved = apart(part.body->pv.position, part.sentPv.position);
			const bool partPushed =
			    apart(part.body->pv.velocity.linear, part.sentPv.velocity.linear, kPushedStudsPerSecond) ||
			    apart(part.body->pv.velocity.rotational, part.sentPv.velocity.rotational, kPushedStudsPerSecond);

			if (moved == nullptr && (partMoved || partPushed) && part.parent < 0 && part.body->parent == nullptr) {
				moved = &part;
				coordChanged = partMoved;
				velocityChanged = partPushed;
			}
		}

		if (moved == nullptr) {
			continue;
		}

		const rbx::PV& pv = moved->body->pv;

		if (coordChanged) {
			const rbx::CoordinateFrame coord = pv.position * rbx::inverse(moved->meInAssembly);

			b3Body_SetTransform(assembly.id, toVec3(coord.translation), toQuat(coord.rotation));
			assembly.publishedCoord = coord;
		}

		if (velocityChanged) {
			const b3Vec3 rotational = toVec3(pv.velocity.rotational);
			const b3Vec3 arm = b3Sub(b3Body_GetWorldCenter(assembly.id), toVec3(pv.position.translation));
			const b3Vec3 linear = b3Add(toVec3(pv.velocity.linear), b3Cross(rotational, arm));

			b3Body_SetLinearVelocity(assembly.id, linear);
			b3Body_SetAngularVelocity(assembly.id, rotational);
		}

		b3Body_SetAwake(assembly.id, true);

		rbx::SimBody* simBody = getSimBody(moved->body);

		if (simBody != nullptr) {
			simBody->dirty = 1;
		}

		moved->sentPv = pv;
	}
}

void Simulation::publishBodies(rbx::World* world, bool notify)
{
	const b3BodyEvents events = b3World_GetBodyEvents(world_);

	for (int i = 0; i < events.moveCount; ++i) {
		const b3BodyMoveEvent& event = events.moveEvents[i];

		auto* assembly = static_cast<Assembly*>(event.userData);

		if (assembly == nullptr || assembly->bodyType == b3_staticBody) {
			continue;
		}

		const b3WorldTransform& moved = event.transform;

		const rbx::CoordinateFrame coord{fromQuat(moved.q), fromVec3(moved.p)};

		assembly->publishedCoord = coord;

		const b3Vec3 center = b3Body_GetWorldCenter(assembly->id);
		const b3Vec3 linear = event.fellAsleep ? b3Vec3_zero : b3Body_GetLinearVelocity(assembly->id);
		const b3Vec3 rotational = event.fellAsleep ? b3Vec3_zero : b3Body_GetAngularVelocity(assembly->id);

		b3Vec3 gained = b3Vec3_zero;

		if (assembly->simOwned && !event.fellAsleep) {
			const b3Quat rotation = moved.q;
			const b3Vec3 delta = b3Sub(rotational, assembly->simRotational);

			gained = b3RotateVector(
			    rotation, b3MulMV(b3Body_GetLocalRotationalInertia(assembly->id), b3InvRotateVector(rotation, delta))
			);
		}

		const std::int32_t stateIndex = rbx::nextStateIndex();

		for (AssemblyPrimitive& part : assembly->primitives) {
			if (!part.live) {
				continue;
			}

			rbx::Body* body = part.body;

			const rbx::CoordinateFrame at = part.parent < 0 ? coord : coord * part.meInAssembly;
			const b3Vec3 arm = b3Sub(toVec3(at.translation), center);

			body->pv.position = at;
			body->pv.velocity.linear = fromVec3(b3Add(linear, b3Cross(rotational, arm)));
			body->pv.velocity.rotational = fromVec3(rotational);
			body->stateIndex = stateIndex;

			rbx::SimBody* simBody = getSimBody(body);

			if (simBody != nullptr) {
				if (assembly->simOwned) {
					const rbx::Vector3 momentum{
					    simBody->angMomentum.x + gained.x,
					    simBody->angMomentum.y + gained.y,
					    simBody->angMomentum.z + gained.z,
					};

					simBody->dirty = 1;
					rbx::updateSimBody(simBody);
					simBody->angMomentum = momentum;
				} else {
					simBody->dirty = 1;
				}
			}

			part.stateIndex = stateIndex;
			part.sentPv = body->pv;

			if (notify) {
				if (apart(at, part.hashedCoord)) {
					part.hashedCoord = at;
					rbx::primitiveExtentsChanged(world, part.primitive);
				}

				rbx::notifyMoved(part.primitive->myOwner);
			}
		}
	}
}

void Simulation::computeFallen(rbx::Array<rbx::Primitive*>& fallen)
{
	std::vector<rbx::Primitive*> found;

	for (const auto& held : assemblies_) {
		const Assembly& assembly = *held;

		if (assembly.bodyType == b3_staticBody || !assembly.primitives.front().live ||
		    !(assembly.primitives.front().body->pv.position.translation.y < rbx::World::fallenY)) {
			continue;
		}

		for (const AssemblyPrimitive& part : assembly.primitives) {
			if (!part.live || !(part.body->pv.position.translation.y < rbx::World::fallenY)) {
				continue;
			}

			if (part.primitive->myOwner == nullptr || reportedFallen_.contains(part.primitive)) {
				continue;
			}

			found.push_back(part.primitive);
		}
	}

	if (found.empty()) {
		return;
	}

	const std::int32_t at = fallen.count;
	const std::int32_t wanted = at + static_cast<std::int32_t>(found.size());

	rbx::resizeArray(&fallen, wanted);

	if (fallen.items == nullptr || fallen.count != wanted || fallen.capacity < wanted) {
		fallen.count = at;
		return;
	}

	for (std::size_t i = 0; i < found.size(); ++i) {
		fallen.items[at + static_cast<std::int32_t>(i)] = found[i];
		reportedFallen_.insert(found[i]);
	}
}

namespace {

void onNewTouch(rbx::World* world, rbx::Primitive* p0, rbx::Primitive* p1)
{
	if (p0 == nullptr || p1 == nullptr) {
		return;
	}

	if (p0->myOwner != nullptr && p0->myOwner->reportTouches()) {
		rbx::onPrimitiveTouched(world, p0, p1);
	}

	if (p1->myOwner != nullptr && p1->myOwner->reportTouches()) {
		rbx::onPrimitiveTouched(world, p1, p0);
	}
}

} // namespace

void Simulation::reportTouches(rbx::World* world)
{
	const b3ContactEvents contacts = b3World_GetContactEvents(world_);

	for (int i = 0; i < contacts.beginCount; ++i) {
		const b3ContactBeginTouchEvent& event = contacts.beginEvents[i];

		onNewTouch(
		    world,
		    static_cast<rbx::Primitive*>(b3Shape_GetUserData(event.shapeIdA)),
		    static_cast<rbx::Primitive*>(b3Shape_GetUserData(event.shapeIdB))
		);
	}

	const b3SensorEvents sensors = b3World_GetSensorEvents(world_);

	for (int i = 0; i < sensors.beginCount; ++i) {
		const b3SensorBeginTouchEvent& event = sensors.beginEvents[i];

		onNewTouch(
		    world,
		    static_cast<rbx::Primitive*>(b3Shape_GetUserData(event.sensorShapeId)),
		    static_cast<rbx::Primitive*>(b3Shape_GetUserData(event.visitorShapeId))
		);
	}
}

} // namespace v8patch::sim
