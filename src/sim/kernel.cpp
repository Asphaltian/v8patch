#include "sim/simulation.h"

#include "rbx/constants.h"
#include "rbx/dynamics.h"
#include "sim/joints.h"
#include "sim/primitives.h"
#include "sim/tracking.h"

namespace v8patch::sim {

bool Simulation::accumulateForces(Assembly& assembly, float dt, bool throttling, bool gate)
{
	bool stepped = false;

	for (AssemblyPrimitive& part : assembly.primitives) {
		if (!part.live) {
			continue;
		}

		rbx::SimBody* simBody = getSimBody(part.body);

		if (simBody == nullptr) {
			continue;
		}

		if (throttling && part.body->canThrottle != 0) {
			rbx::resetAccumulators(simBody);
			continue;
		}

		if (simBody->dirty != 0) {
			rbx::updateSimBody(simBody);
		}

		if (gate) {
			const rbx::Vector3& force = simBody->force;

			if (force.x == 0.0F && force.z == 0.0F && force.y == simBody->constantForceY &&
			    b3LengthSquared(toVec3(simBody->torque)) == 0.0F) {
				continue;
			}
		}

		rbx::stepSimBody(simBody, dt);
		rbx::publishRoot(part.body);

		part.sentPv = part.body->pv;
		part.stateIndex = part.body->stateIndex;

		assembly.simLinear = toVec3(simBody->pv.velocity.linear);
		assembly.simRotational = toVec3(simBody->pv.velocity.rotational);
		stepped = true;
	}

	return stepped;
}

void Simulation::stepKernel(rbx::KernelData* data, bool throttling, bool sweeping)
{
	const int kernelSteps = rbx::Constants::kernelStepsPerWorldStep();
	const float dt = rbx::Constants::kernelDt();

	const bool hasSecondPass = data != nullptr && data->connectors2ndPass.size() > 0;

	for (Assembly* assembly : forced_) {
		assembly->simOwned = false;
	}

	forced_.clear();

	for (int step = 0; step < kernelSteps; ++step) {
		if (hasSecondPass) {
			for (rbx::Connector* connector : data->connectors2ndPass) {
				if (connector != nullptr) {
					connector->computeForce(dt, throttling);
				}
			}
		}

		if (step > 0) {
			for (std::size_t i = 0; i < forced_.size(); ++i) {
				accumulateForces(*forced_[i], dt, throttling, false);
			}

			continue;
		}

		if (sweeping) {
			for (const auto& held : assemblies_) {
				if (held->bodyType != b3_dynamicBody || held->jointed) {
					continue;
				}

				if (accumulateForces(*held, dt, throttling, true)) {
					forced_.push_back(held.get());
				}
			}

			continue;
		}

		awakeIds_.resize(assemblies_.size() + 1);

		const int count = b3World_GetAwakeBodies(world_, awakeIds_.data(), static_cast<int>(awakeIds_.size()));

		for (int i = 0; i < count; ++i) {
			auto* assembly = static_cast<Assembly*>(b3Body_GetUserData(awakeIds_[static_cast<std::size_t>(i)]));

			if (assembly == nullptr || assembly->bodyType != b3_dynamicBody || assembly->jointed) {
				continue;
			}

			if (accumulateForces(*assembly, dt, throttling, true)) {
				forced_.push_back(assembly);
			}
		}
	}

	for (Assembly* assembly : forced_) {
		assembly->simOwned = true;

		b3Body_SetAwake(assembly->id, true);
		b3Body_SetLinearVelocity(assembly->id, b3Sub(assembly->simLinear, gravityStep_));
		b3Body_SetAngularVelocity(assembly->id, b3MulSV(spinRegain_, assembly->simRotational));
	}
}

} // namespace v8patch::sim
