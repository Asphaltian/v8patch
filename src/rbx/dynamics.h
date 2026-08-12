#pragma once

#include "rbx/engine.h"

namespace v8patch::rbx {

inline constexpr float kAngularRetention = 0.9998F;

[[nodiscard]] CoordinateFrame operator*(const CoordinateFrame& outer, const CoordinateFrame& inner) noexcept;
[[nodiscard]] CoordinateFrame inverse(const CoordinateFrame& frame) noexcept;
[[nodiscard]] CoordinateFrame rotateAboutZ(const CoordinateFrame& frame, float radians) noexcept;

void resetAccumulators(SimBody* simBody) noexcept;
void stepSimBody(SimBody* simBody, float dt) noexcept;
void publishRoot(Body* body) noexcept;

} // namespace v8patch::rbx
