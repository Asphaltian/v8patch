#pragma once

#include <bit>
#include <cmath>
#include <cstdint>

#include "rbx/engine.h"

namespace v8patch::sim {

constexpr float kMovedStuds = 1.0e-3F;
constexpr float kMovedRotation = 1.0e-4F;
constexpr float kPushedStudsPerSecond = 1.0e-2F;

constexpr std::int32_t kScanEvery = 16;

constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

inline std::uint64_t hashOf(const void* pointer) noexcept
{
	std::uint64_t value = reinterpret_cast<std::uintptr_t>(pointer);

	value += 0x9e3779b97f4a7c15ULL;
	value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
	value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;

	return value ^ (value >> 31);
}

inline std::uint64_t mixInto(std::uint64_t seed, std::uint64_t value) noexcept
{
	return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

inline std::uint64_t clumpHashOf(const rbx::Primitive* primitive) noexcept
{
	std::uint64_t value = hashOf(primitive);

	if (primitive == nullptr) {
		return value;
	}

	value += hashOf(primitive->geometry);

	if (primitive->body != nullptr) {
		value += std::bit_cast<std::uint32_t>(primitive->body->mass);
	}

	if (primitive->geometry != nullptr) {
		const rbx::Vector3& gridSize = primitive->geometry->gridSize;

		value += std::bit_cast<std::uint32_t>(gridSize.x);
		value += std::bit_cast<std::uint32_t>(gridSize.y);
		value += std::bit_cast<std::uint32_t>(gridSize.z);
	}

	return value;
}

inline std::uint64_t signatureOf(const rbx::Primitive* primitive, std::uint64_t seed) noexcept
{
	return mixInto(seed, hashOf(primitive));
}

inline bool apart(const rbx::Vector3& first, const rbx::Vector3& second, float tolerance) noexcept
{
	return std::fabs(first.x - second.x) > tolerance || std::fabs(first.y - second.y) > tolerance ||
	       std::fabs(first.z - second.z) > tolerance;
}

inline bool
apart(const rbx::CoordinateFrame& first, const rbx::CoordinateFrame& second, float studs, float rotation) noexcept
{
	if (apart(first.translation, second.translation, studs)) {
		return true;
	}

	for (std::size_t i = 0; i < first.rotation.row.size(); ++i) {
		if (std::fabs(first.rotation.row[i] - second.rotation.row[i]) > rotation) {
			return true;
		}
	}

	return false;
}

inline bool apart(const rbx::CoordinateFrame& first, const rbx::CoordinateFrame& second) noexcept
{
	return apart(first, second, kMovedStuds, kMovedRotation);
}

} // namespace v8patch::sim
