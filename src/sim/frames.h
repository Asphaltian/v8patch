#pragma once

#include <box3d/box3d.h>

#include "rbx/dynamics.h"
#include "rbx/engine.h"

namespace v8patch::sim {

[[nodiscard]] inline b3Vec3 toVec3(const rbx::Vector3& value) noexcept
{
	return b3Vec3{value.x, value.y, value.z};
}

[[nodiscard]] inline rbx::Vector3 fromVec3(b3Vec3 value) noexcept
{
	return rbx::Vector3{value.x, value.y, value.z};
}

[[nodiscard]] inline b3Matrix3 toMatrix3(const rbx::Matrix3& value) noexcept
{
	return b3Matrix3{
	    b3Vec3{value.row[0], value.row[3], value.row[6]},
	    b3Vec3{value.row[1], value.row[4], value.row[7]},
	    b3Vec3{value.row[2], value.row[5], value.row[8]},
	};
}

[[nodiscard]] inline rbx::Matrix3 fromMatrix3(const b3Matrix3& value) noexcept
{
	return rbx::Matrix3{{
	    value.cx.x,
	    value.cy.x,
	    value.cz.x,
	    value.cx.y,
	    value.cy.y,
	    value.cz.y,
	    value.cx.z,
	    value.cy.z,
	    value.cz.z,
	}};
}

[[nodiscard]] inline b3Quat toQuat(const rbx::Matrix3& value) noexcept
{
	const b3Matrix3 columns = toMatrix3(value);

	return b3MakeQuatFromMatrix(&columns);
}

[[nodiscard]] inline rbx::Matrix3 fromQuat(b3Quat value) noexcept
{
	return fromMatrix3(b3MakeMatrixFromQuat(value));
}

[[nodiscard]] inline b3Transform toTransform(const rbx::CoordinateFrame& frame) noexcept
{
	return b3Transform{toVec3(frame.translation), toQuat(frame.rotation)};
}

} // namespace v8patch::sim
