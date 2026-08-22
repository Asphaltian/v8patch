#include "sim/simulation.h"

#include <algorithm>
#include <cmath>

#include "log.h"
#include "rbx/dynamics.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

namespace v8patch::sim {

AssemblyPrimitive* Simulation::getAssemblyPrimitive(const rbx::Primitive* primitive) noexcept
{
	const std::int32_t worldIndex = primitive->worldIndex;

	if (worldIndex < 0 || static_cast<std::size_t>(worldIndex) >= primitiveToPart_.size()) {
		return nullptr;
	}

	AssemblyPrimitive* part = primitiveToPart_[static_cast<std::size_t>(worldIndex)];

	return part != nullptr && part->primitive == primitive ? part : nullptr;
}

void Simulation::findClumps(const rbx::Array<rbx::Primitive*>& primitives)
{
	const std::int32_t count = primitives.size();

	clumps_.reset(count);
	connectorJoints_.clear();

	for (std::int32_t i = 0; i < count; ++i) {
		rbx::Primitive* primitive = primitives.items[i];

		if (primitive == nullptr || primitive->body == nullptr) {
			clumps_.drop(i);
			continue;
		}

		if (getAnchor(primitive)) {
			clumps_.anchor(i);
		}

		rbx::Edge* edge = primitive->firstJoint();

		for (int guard = 0; edge != nullptr && guard < kMaxJointsPerPrimitive; ++guard) {
			rbx::Edge* next = edge->next(primitive);
			rbx::Primitive* other = edge->other(primitive);
			auto* joint = static_cast<rbx::Joint*>(edge);

			if (other != nullptr && other->body != nullptr) {
				const rbx::Joint::Type jointType = joint->jointType();
				const std::int32_t j = other->worldIndex;

				if (isRigidJoint(jointType) && j >= 0 && j < count && primitives.items[j] == other) {
					clumps_.merge(i, j);
				} else if (edge->prim0 == primitive && isConnectorJoint(jointType)) {
					connectorJoints_.push_back(joint);
				}
			}

			edge = next;
		}
	}

	clumps_.gather();
}

void Simulation::cleanAssemblies(const rbx::Array<rbx::Primitive*>& primitives)
{
	byHash_.clear();
	byHash_.reserve(assemblies_.size() * 2);
	wasStatic_.clear();

	for (const auto& held : assemblies_) {
		held->clean = false;
		byHash_.emplace(held->primitiveHash, held.get());

		if (held->bodyType != b3_staticBody) {
			continue;
		}

		for (const AssemblyPrimitive& part : held->primitives) {
			wasStatic_.insert(part.primitive);
		}
	}

	dirtyClumps_.clear();
	forced_.clear();

	for (std::size_t at = 0; at < clumps_.groupCount(); ++at) {
		const std::span<const std::int32_t> members = clumps_.group(at);

		std::uint64_t primitiveHash = 0;

		for (std::int32_t member : members) {
			primitiveHash += clumpHashOf(primitives.items[member]);
		}

		const auto known = byHash_.find(primitiveHash);

		if (known != byHash_.end() && known->second->primitives.size() == members.size()) {
			known->second->clean = true;
			byHash_.erase(known);
		} else {
			dirtyClumps_.push_back(at);
		}
	}

	const auto stale =
	    std::partition(assemblies_.begin(), assemblies_.end(), [](const auto& held) { return held->clean; });

	for (auto it = stale; it != assemblies_.end(); ++it) {
		b3DestroyBody((*it)->id);
	}

	assemblies_.erase(stale, assemblies_.end());

	for (std::size_t at : dirtyClumps_) {
		createAssembly(primitives, at);
	}

	unresolved_.clear();

	for (const auto& held : assemblies_) {
		if (held->unresolved) {
			unresolved_.push_back(held.get());
		}
	}

	liveJoints_.clear();
	liveJoints_.insert(connectorJoints_.begin(), connectorJoints_.end());

	std::erase_if(connectors_, [this](auto& entry) {
		if (liveJoints_.contains(entry.first) && (entry.second.broken || b3Joint_IsValid(entry.second.id))) {
			return false;
		}

		removeConnector(entry.second);
		return true;
	});

	primitiveToPart_.assign(static_cast<std::size_t>(primitives.size()), nullptr);

	for (const auto& held : assemblies_) {
		Assembly& assembly = *held;

		for (AssemblyPrimitive& part : assembly.primitives) {
			part.live = livePrimitives_.contains(part.primitive);

			const std::int32_t worldIndex = part.primitive->worldIndex;

			if (worldIndex >= 0 && static_cast<std::size_t>(worldIndex) < primitiveToPart_.size()) {
				primitiveToPart_[static_cast<std::size_t>(worldIndex)] = &part;
			}
		}
	}

	std::erase_if(reportedFallen_, [this](const rbx::Primitive* primitive) {
		return !livePrimitives_.contains(primitive);
	});
}

bool Simulation::isMotorChild(const rbx::Primitive* primitive) const
{
	const rbx::Edge* edge = primitive->firstJoint();

	for (int guard = 0; edge != nullptr && guard < kMaxJointsPerPrimitive; ++guard) {
		const auto* joint = static_cast<const rbx::Joint*>(edge);

		if (edge->prim1 == primitive && joint->jointType() == rbx::Joint::Motor && clumpSlot_.contains(edge->prim0)) {
			return true;
		}

		edge = edge->next(primitive);
	}

	return false;
}

void Simulation::createAssembly(const rbx::Array<rbx::Primitive*>& primitives, std::size_t at)
{
	const std::span<const std::int32_t> members = clumps_.group(at);
	const bool anchored = clumps_.anchored(at);

	clumpSlot_.clear();
	clumpSlot_.reserve(members.size() * 2);

	for (std::int32_t member : members) {
		clumpSlot_.emplace(primitives.items[member], -1);
	}

	for (std::int32_t member : members) {
		rbx::Primitive* assemblyPrimitive = primitives.items[member];

		if (clumpSlot_[assemblyPrimitive] < 0 && assemblyPrimitive->body->parent == nullptr) {
			buildAssembly(assemblyPrimitive, anchored);
		}
	}

	for (std::int32_t member : members) {
		rbx::Primitive* assemblyPrimitive = primitives.items[member];

		if (clumpSlot_[assemblyPrimitive] < 0 && !isMotorChild(assemblyPrimitive)) {
			buildAssembly(assemblyPrimitive, anchored);
		}
	}

	for (std::int32_t member : members) {
		rbx::Primitive* assemblyPrimitive = primitives.items[member];

		if (clumpSlot_[assemblyPrimitive] < 0) {
			buildAssembly(assemblyPrimitive, anchored);
		}
	}
}

void Simulation::buildAssembly(rbx::Primitive* assemblyPrimitive, bool anchored)
{
	auto held = std::make_unique<Assembly>();
	Assembly& assembly = *held;

	assembly.anchored = anchored;
	assembly.clean = true;
	assembly.primitives.push_back(AssemblyPrimitive{.primitive = assemblyPrimitive, .body = assemblyPrimitive->body});
	clumpSlot_[assemblyPrimitive] = 0;

	for (std::size_t head = 0; head < assembly.primitives.size(); ++head) {
		rbx::Primitive* primitive = assembly.primitives[head].primitive;
		rbx::Edge* edge = primitive->firstJoint();

		for (int guard = 0; edge != nullptr && guard < kMaxJointsPerPrimitive; ++guard) {
			rbx::Edge* next = edge->next(primitive);
			rbx::Primitive* other = edge->other(primitive);
			auto* joint = static_cast<rbx::Joint*>(edge);
			const rbx::Joint::Type jointType = joint->jointType();

			if (other != nullptr && other->body != nullptr && isRigidJoint(jointType)) {
				const auto slot = clumpSlot_.find(other);

				if (slot != clumpSlot_.end() && slot->second < 0) {
					slot->second = static_cast<std::int32_t>(assembly.primitives.size());

					AssemblyPrimitive child{.primitive = other, .body = other->body};
					child.parent = static_cast<std::int32_t>(head);
					child.parentIsPrim0 = edge->prim0 == primitive;
					child.motor = jointType == rbx::Joint::Motor ? static_cast<rbx::MotorJoint*>(joint) : nullptr;

					assembly.hasMotor = assembly.hasMotor || child.motor != nullptr;

					const float angle = child.motor != nullptr ? child.motor->currentAngle : 0.0F;
					const rbx::CoordinateFrame childInParent = computeChildInParent(joint, child.parentIsPrim0, angle);

					child.meInAssembly = assembly.primitives[head].meInAssembly * childInParent;

					assembly.primitives.push_back(child);
				}
			}

			edge = next;
		}
	}

	bool canSleep = true;

	for (AssemblyPrimitive& part : assembly.primitives) {
		part.assembly = &assembly;
		canSleep = canSleep && part.primitive->canSleep != 0;
		part.partMass = part.body->mass;
		assembly.primitiveHash += clumpHashOf(part.primitive);
	}

	const AssemblyPrimitive& root = assembly.primitives.front();
	const rbx::CoordinateFrame coord = root.body->pv.position;

	assembly.bodyType = getBodyType(assembly);

	b3BodyDef def = b3DefaultBodyDef();
	def.type = assembly.bodyType;
	def.position = toVec3(coord.translation);
	def.rotation = toQuat(coord.rotation);
	def.angularVelocity = toVec3(root.body->pv.velocity.rotational);
	def.angularDamping = angularDamping();
	def.enableSleep = canSleep;
	def.allowFastRotation = true;
	def.sleepThreshold = kSleepVelocity;
	def.isAwake = def.type != b3_staticBody;
	def.userData = &assembly;

	assembly.id = b3CreateBody(world_, &def);
	assembly.publishedCoord = coord;

	for (AssemblyPrimitive& part : assembly.primitives) {
		createGeometry(assembly, part);

		rbx::SimBody* simBody = getSimBody(part.body);

		if (simBody != nullptr && wasStatic_.contains(part.primitive)) {
			rbx::resetAccumulators(simBody);
		}

		part.stateIndex = part.body->stateIndex;
		part.sentPv = part.body->pv;
		part.hashedCoord = coord * part.meInAssembly;
	}

	if (assembly.bodyType != b3_staticBody) {
		setBranchMass(assembly);

		const b3Vec3 rotational = toVec3(root.body->pv.velocity.rotational);
		const b3Vec3 arm = b3Sub(b3Body_GetWorldCenter(assembly.id), toVec3(coord.translation));

		b3Body_SetLinearVelocity(assembly.id, b3Add(toVec3(root.body->pv.velocity.linear), b3Cross(rotational, arm)));
	}

	assembly.unresolved = false;

	if (assembly.bodyType != b3_staticBody) {
		for (const AssemblyPrimitive& part : assembly.primitives) {
			if (!(part.body->mass > 0.0F) || part.builtType == rbx::Geometry::None) {
				assembly.unresolved = true;
				break;
			}
		}
	}

	assemblies_.push_back(std::move(held));
}

bool Simulation::resolved() const noexcept
{
	for (const Assembly* assembly : unresolved_) {
		for (const AssemblyPrimitive& part : assembly->primitives) {
			if (part.body->mass > 0.0F && !(part.partMass > 0.0F)) {
				return false;
			}

			if (part.builtType == rbx::Geometry::None && part.primitive->geometry != nullptr) {
				return false;
			}
		}
	}

	return true;
}

void Simulation::createGeometry(Assembly& assembly, AssemblyPrimitive& part)
{
	if (b3Shape_IsValid(part.shape)) {
		b3DestroyShape(part.shape, false);
		part.shape = b3_nullShapeId;
	}

	part.canCollide = getCanCollide(part.primitive);

	const rbx::Geometry* geometry = part.primitive->geometry;
	const rbx::Geometry::Type geometryType = geometry != nullptr ? geometry->type() : rbx::Geometry::None;
	const float gridVolume = geometry != nullptr ? geometry->gridVolume() : 0.0F;

	part.builtType = geometryType;
	part.builtSize = geometry != nullptr ? geometry->gridSize : rbx::Vector3{};

	if ((geometryType != rbx::Geometry::Ball && geometryType != rbx::Geometry::Block) || !(gridVolume > 0.0F)) {
		return;
	}

	b3ShapeDef def = b3DefaultShapeDef();
	def.baseMaterial.friction = std::max(0.0F, part.primitive->friction);

	def.baseMaterial.restitution = std::max(0.0F, part.primitive->elasticity);
	def.baseMaterial.jointK = rbx::jointK(part.primitive);
	def.density = std::max(kLeastDensity, part.partMass / gridVolume);
	def.updateBodyMass = false;
	def.enableContactEvents = true;
	def.enableSensorEvents = true;
	def.enableCustomFiltering = true;
	def.isSensor = !part.canCollide;
	def.userData = part.primitive;

	const b3Transform at = toTransform(part.meInAssembly);

	if (geometryType == rbx::Geometry::Ball) {
		const b3Sphere sphere{at.p, std::max(kLeastExtent, geometry->radius())};
		part.shape = b3CreateSphereShape(assembly.id, &def, &sphere);
	} else {
		const b3BoxHull hull = makeHull(part.builtSize, at);
		part.shape = b3CreateHullShape(assembly.id, &def, &hull.base);
	}
}

void Simulation::setShape(AssemblyPrimitive& part)
{
	const b3Transform at = toTransform(part.meInAssembly);

	if (part.builtType == rbx::Geometry::Ball) {
		const b3Sphere sphere{at.p, std::max(kLeastExtent, part.primitive->geometry->radius())};
		b3Shape_SetSphere(part.shape, &sphere);
		return;
	}

	const b3BoxHull hull = makeHull(part.builtSize, at);
	b3Shape_SetHull(part.shape, &hull.base);
}

void Simulation::setBodyType(Assembly& assembly)
{
	const b3BodyType bodyType = getBodyType(assembly);

	if (bodyType == assembly.bodyType) {
		return;
	}

	assembly.bodyType = bodyType;
	b3Body_SetType(assembly.id, bodyType);

	if (bodyType == b3_staticBody) {
		return;
	}

	for (AssemblyPrimitive& part : assembly.primitives) {
		rbx::SimBody* simBody = getSimBody(part.body);

		if (simBody != nullptr) {
			rbx::resetAccumulators(simBody);
		}
	}

	setBranchMass(assembly);
	b3Body_SetAwake(assembly.id, true);
}

void Simulation::setBranchMass(Assembly& assembly)
{
	float mass = 0.0F;
	b3Vec3 weighted = b3Vec3_zero;

	for (const AssemblyPrimitive& part : assembly.primitives) {
		mass += part.body->mass;
		weighted = b3Add(weighted, b3MulSV(part.body->mass, toVec3(part.meInAssembly.translation)));
	}

	if (!(mass > 0.0F)) {
		b3Body_ApplyMassFromShapes(assembly.id);
		return;
	}

	const b3Vec3 centre = b3MulSV(1.0F / mass, weighted);

	b3Matrix3 inertia = b3Mat3_zero;

	for (const AssemblyPrimitive& part : assembly.primitives) {
		const b3Matrix3 rotation = toMatrix3(part.meInAssembly.rotation);
		const b3Matrix3 own = b3MulMM(b3MulMM(rotation, toMatrix3(part.body->moment)), b3Transpose(rotation));
		const b3Vec3 arm = b3Sub(toVec3(part.meInAssembly.translation), centre);

		inertia = b3AddMM(inertia, b3AddMM(own, b3Steiner(part.body->mass, arm)));
	}

	const b3MassData massData{
	    mass,
	    centre,
	    b3Matrix3{
	        b3Vec3{inertia.cx.x, 0.0F, 0.0F},
	        b3Vec3{0.0F, inertia.cy.y, 0.0F},
	        b3Vec3{0.0F, 0.0F, inertia.cz.z},
	    },
	};

	b3Body_SetMassData(assembly.id, massData);

	for (AssemblyPrimitive& part : assembly.primitives) {
		rbx::SimBody* simBody = getSimBody(part.body);

		if (simBody != nullptr) {
			simBody->dirty = 1;
		}
	}
}

void Simulation::stepUi()
{
	std::uint64_t anchors = 0;

	for (auto& held : assemblies_) {
		Assembly& assembly = *held;

		bool changed = false;

		for (AssemblyPrimitive& part : assembly.primitives) {
			if (!part.live) {
				continue;
			}

			const rbx::Geometry* geometry = part.primitive->geometry;

			if (getAnchor(part.primitive)) {
				anchors += hashOf(part.primitive);
			}

			if (geometry != nullptr && geometry->gridSize != part.builtSize) {
				part.partMass = part.body->mass;
			} else if (getCanCollide(part.primitive) == part.canCollide && geometry != nullptr &&
			           geometry->type() == part.builtType) {
				continue;
			}

			createGeometry(assembly, part);
			changed = true;
		}

		if (assembly.hasMotor) {
			for (std::size_t k = 1; k < assembly.primitives.size(); ++k) {
				AssemblyPrimitive& part = assembly.primitives[k];

				if (part.motor == nullptr || part.parent < 0) {
					continue;
				}

				const float angle = stepUiAngle(part.motor);
				const rbx::CoordinateFrame childInParent = computeChildInParent(part.motor, part.parentIsPrim0, angle);
				const rbx::CoordinateFrame& parent =
				    assembly.primitives[static_cast<std::size_t>(part.parent)].meInAssembly;
				const rbx::CoordinateFrame meInAssembly = parent * childInParent;

				if (meInAssembly == part.meInAssembly) {
					continue;
				}

				part.meInAssembly = meInAssembly;
				changed = true;

				const rbx::CoordinateFrame at = assembly.publishedCoord * meInAssembly;

				part.body->pv.position = at;
				part.sentPv.position = at;
				part.stateIndex = part.body->stateIndex;

				if (b3Shape_IsValid(part.shape)) {
					setShape(part);
				}
			}
		}

		setBodyType(assembly);

		if (changed && assembly.bodyType != b3_staticBody) {
			setBranchMass(assembly);
			b3Body_SetAwake(assembly.id, true);
		}
	}

	if (anchors != anchors_) {
		anchors_ = anchors;
		jointCount_ = -1;
	}
}

} // namespace v8patch::sim
