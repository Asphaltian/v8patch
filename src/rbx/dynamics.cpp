#include "rbx/dynamics.h"

#include <cmath>

namespace v8patch::rbx {
namespace {

float& at(Matrix3& matrix, int row, int column) noexcept
{
	return matrix.row[static_cast<std::size_t>(row) * 3 + static_cast<std::size_t>(column)];
}

float at(const Matrix3& matrix, int row, int column) noexcept
{
	return matrix.row[static_cast<std::size_t>(row) * 3 + static_cast<std::size_t>(column)];
}

float& at(Vector3& vector, int index) noexcept
{
	return (&vector.x)[index];
}

float at(const Vector3& vector, int index) noexcept
{
	return (&vector.x)[index];
}

} // namespace

CoordinateFrame operator*(const CoordinateFrame& outer, const CoordinateFrame& inner) noexcept
{
	CoordinateFrame out{};

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			at(out.rotation, row, column) = at(outer.rotation, row, 0) * at(inner.rotation, 0, column) +
			                                at(outer.rotation, row, 1) * at(inner.rotation, 1, column) +
			                                at(outer.rotation, row, 2) * at(inner.rotation, 2, column);
		}

		at(out.translation, row) = at(outer.translation, row) + at(outer.rotation, row, 0) * inner.translation.x +
		                           at(outer.rotation, row, 1) * inner.translation.y +
		                           at(outer.rotation, row, 2) * inner.translation.z;
	}

	return out;
}

CoordinateFrame inverse(const CoordinateFrame& frame) noexcept
{
	CoordinateFrame out{};

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			at(out.rotation, row, column) = at(frame.rotation, column, row);
		}

		at(out.translation, row) = -(at(out.rotation, row, 0) * frame.translation.x +
		                             at(out.rotation, row, 1) * frame.translation.y +
		                             at(out.rotation, row, 2) * frame.translation.z);
	}

	return out;
}

CoordinateFrame rotateAboutZ(const CoordinateFrame& frame, float radians) noexcept
{
	const float sinR = std::sin(radians);
	const float cosR = std::cos(radians);

	CoordinateFrame out{{}, frame.translation};

	for (int row = 0; row < 3; ++row) {
		const float x = at(frame.rotation, row, 0);
		const float y = at(frame.rotation, row, 1);

		at(out.rotation, row, 0) = x * cosR + y * sinR;
		at(out.rotation, row, 1) = y * cosR - x * sinR;
		at(out.rotation, row, 2) = at(frame.rotation, row, 2);
	}

	return out;
}

void resetAccumulators(SimBody* simBody) noexcept
{
	if (simBody->dirty != 0) {
		updateSimBody(simBody);
	}

	simBody->clearAccumulators();
}

namespace {

constexpr float kDenormFix = 1.0e-20F;

Vector3 cross(const Vector3& first, const Vector3& second) noexcept
{
	return Vector3{
	    first.y * second.z - first.z * second.y,
	    first.z * second.x - first.x * second.z,
	    first.x * second.y - first.y * second.x,
	};
}

Vector3 computeRotVel(const Matrix3& rotation, const Vector3& momentRecip, const Vector3& angMomentum) noexcept
{
	Matrix3 scaled{};

	for (int row = 0; row < 3; ++row) {
		at(scaled, row, 0) = at(rotation, row, 0) * momentRecip.x;
		at(scaled, row, 1) = at(rotation, row, 1) * momentRecip.y;
		at(scaled, row, 2) = at(rotation, row, 2) * momentRecip.z;
	}

	Matrix3 inverseWorld{};

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			at(inverseWorld, row, column) = at(scaled, row, 0) * at(rotation, column, 0) +
			                                at(scaled, row, 1) * at(rotation, column, 1) +
			                                at(scaled, row, 2) * at(rotation, column, 2);
		}
	}

	Vector3 rotVel{};

	for (int row = 0; row < 3; ++row) {
		at(rotVel, row) = at(inverseWorld, row, 0) * angMomentum.x + at(inverseWorld, row, 1) * angMomentum.y +
		                  at(inverseWorld, row, 2) * angMomentum.z;
	}

	return rotVel;
}

Quaternion multiply(const Quaternion& first, const Quaternion& second) noexcept
{
	const Vector3 left{first.x, first.y, first.z};
	const Vector3 right{second.x, second.y, second.z};
	const Vector3 turn = cross(left, right);

	return Quaternion{
	    right.x * first.w + left.x * second.w + turn.x,
	    right.y * first.w + left.y * second.w + turn.y,
	    right.z * first.w + left.z * second.w + turn.z,
	    first.w * second.w - (right.x * left.x + right.y * left.y + right.z * left.z),
	};
}

void toRotationMatrix(const Quaternion& quaternion, Matrix3& rotation) noexcept
{
	const float tx = quaternion.x * 2.0F;
	const float ty = quaternion.y * 2.0F;
	const float tz = quaternion.z * 2.0F;

	const float twx = quaternion.x * tx;
	const float txy = quaternion.x * ty;
	const float txz = quaternion.x * tz;
	const float twxw = quaternion.w * tx;
	const float twy = quaternion.w * ty;
	const float wz = quaternion.w * tz;
	const float tyy = quaternion.y * ty;
	const float yz = quaternion.y * tz;
	const float zz = quaternion.z * tz;

	at(rotation, 0, 0) = 1.0F - (zz + tyy);
	at(rotation, 0, 1) = txy - wz;
	at(rotation, 0, 2) = twy + txz;
	at(rotation, 1, 0) = wz + txy;
	at(rotation, 1, 1) = 1.0F - (zz + twx);
	at(rotation, 1, 2) = yz - twxw;
	at(rotation, 2, 0) = txz - twy;
	at(rotation, 2, 1) = yz + twxw;
	at(rotation, 2, 2) = 1.0F - (tyy + twx);
}

} // namespace

void stepSimBody(SimBody* simBody, float dt) noexcept
{
	if (simBody->dirty != 0) {
		updateSimBody(simBody);
	}

	Vector3& angMomentum = simBody->angMomentum;

	angMomentum.x = angMomentum.x * kAngularRetention + simBody->torque.x * dt;
	angMomentum.y = angMomentum.y * kAngularRetention + simBody->torque.y * dt;
	angMomentum.z = angMomentum.z * kAngularRetention + simBody->torque.z * dt;

	PV& pv = simBody->pv;

	pv.velocity.rotational = computeRotVel(pv.position.rotation, simBody->momentRecip, angMomentum);
	pv.velocity.rotational.x += kDenormFix;
	pv.velocity.rotational.y += kDenormFix;
	pv.velocity.rotational.z += kDenormFix;

	Quaternion& orientation = simBody->qOrientation;
	const Quaternion spin{pv.velocity.rotational.x, pv.velocity.rotational.y, pv.velocity.rotational.z, 0.0F};
	const Quaternion delta = multiply(spin, orientation);
	const float scale = 0.5F * dt;

	orientation.x += delta.x * scale;
	orientation.y += delta.y * scale;
	orientation.z += delta.z * scale;
	orientation.w += delta.w * scale;

	const float length = std::sqrt(
	    orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z +
	    orientation.w * orientation.w
	);

	if (length > 0.0F) {
		const float recip = 1.0F / length;

		orientation.x *= recip;
		orientation.y *= recip;
		orientation.z *= recip;
		orientation.w *= recip;
	}

	toRotationMatrix(orientation, pv.position.rotation);

	pv.velocity.linear.x += simBody->force.x * simBody->massRecip * dt;
	pv.velocity.linear.y += simBody->force.y * simBody->massRecip * dt;
	pv.velocity.linear.z += simBody->force.z * simBody->massRecip * dt;

	pv.position.translation.x += pv.velocity.linear.x * dt;
	pv.position.translation.y += pv.velocity.linear.y * dt;
	pv.position.translation.z += pv.velocity.linear.z * dt;

	simBody->clearAccumulators();

	pv.velocity.linear.x += kDenormFix;
	pv.velocity.linear.y += kDenormFix;
	pv.velocity.linear.z += kDenormFix;

	angMomentum.x += kDenormFix;
	angMomentum.y += kDenormFix;
	angMomentum.z += kDenormFix;
}

void publishRoot(Body* body) noexcept
{
	const PV& pv = body->simBody->pv;
	Cofm* cofm = body->cofm;

	body->pv.position.rotation = pv.position.rotation;
	body->stateIndex = nextStateIndex();

	if (cofm == nullptr || cofm->body != body) {
		body->pv.position.translation = pv.position.translation;
		body->pv.velocity = pv.velocity;

		return;
	}

	Vector3 arm{};

	for (int row = 0; row < 3; ++row) {
		at(arm, row) = -(at(pv.position.rotation, row, 0) * cofm->cofmInBody.x +
		                 at(pv.position.rotation, row, 1) * cofm->cofmInBody.y +
		                 at(pv.position.rotation, row, 2) * cofm->cofmInBody.z);
	}

	body->pv.position.translation.x = pv.position.translation.x + arm.x;
	body->pv.position.translation.y = pv.position.translation.y + arm.y;
	body->pv.position.translation.z = pv.position.translation.z + arm.z;

	body->pv.velocity.rotational = pv.velocity.rotational;
	body->pv.velocity.linear.x = pv.velocity.linear.x + (pv.velocity.rotational.y * arm.z - pv.velocity.rotational.z * arm.y);
	body->pv.velocity.linear.y = pv.velocity.linear.y + (pv.velocity.rotational.z * arm.x - pv.velocity.rotational.x * arm.z);
	body->pv.velocity.linear.z = pv.velocity.linear.z + (pv.velocity.rotational.x * arm.y - pv.velocity.rotational.y * arm.x);
}

} // namespace v8patch::rbx
