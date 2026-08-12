#include "rbx/constants.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace v8patch::rbx {
namespace {

constexpr std::array<float, 7> kMaxLegoJointForcesMeasured{0.0F, 1.098F, 2.134F, 2.427F, 3.191F, 4.571F, 4.681F};

constexpr float kJointKScale = 960000.0F;
constexpr float kLegoForceScale = 7500.0F;

int truncate(float value) noexcept
{
	return static_cast<int>(value);
}

int round(float value) noexcept
{
	return static_cast<int>(std::floor(value + 0.5F));
}

Vector3 sortVector3(const Vector3& value) noexcept
{
	std::array<float, 3> sorted{value.x, value.y, value.z};
	std::sort(sorted.begin(), sorted.end());

	return Vector3{sorted[0], sorted[1], sorted[2]};
}

} // namespace

float Constants::jointKMultiplier(const Vector3& clippedSortedSize, bool ball) noexcept
{
	const int x = truncate(clippedSortedSize.x);
	const int y = truncate(clippedSortedSize.y);
	const int z = truncate(clippedSortedSize.z);

	if (ball) {
		switch (x) {
		case 1:
			return 0.23F;
		case 2:
			return 1.49F;
		case 3:
			return 4.43F;
		case 4:
			return 11.5F;
		default:
			return static_cast<float>(x * x * x) * 0.175F;
		}
	}

	switch (x) {
	case 1:
		switch (y) {
		case 1:
			switch (z) {
			case 1:
				return 0.91F;
			case 2:
				return 1.61F;
			case 3:
				return 2.0F;
			case 4:
				return 2.13F;
			default:
				return static_cast<float>(z) * 0.4F;
			}
		case 2:
			switch (z) {
			case 2:
				return 3.5F;
			case 3:
				return 4.16F;
			case 4:
				return 4.79F;
			default:
				break;
			}

			return static_cast<float>(z) < 15.0F ? static_cast<float>(z) * 0.9F : static_cast<float>(z) * 0.75F;
		case 3:
			return static_cast<float>(z) < 7.0F ? static_cast<float>(z) * 1.66F : static_cast<float>(z) * 1.18F;
		case 4:
			return static_cast<float>(z) < 7.0F ? static_cast<float>(z) * 2.26F : static_cast<float>(z) * 1.53F;
		default:
			return (static_cast<float>(y) * 0.3F + 0.66F) * static_cast<float>(z);
		}
	case 2:
		break;
	default:
		return static_cast<float>(z * y * x) * 0.25F;
	}

	float value = 0.0F;

	switch (y) {
	case 2:
		switch (z) {
		case 2:
			return 7.34F;
		case 3:
			return 9.9F;
		case 4:
			return 11.22F;
		default:
			break;
		}

		value = static_cast<float>(z);

		if (value < 15.0F) {
			return value * 1.9F;
		}

		break;
	case 3:
		switch (z) {
		case 3:
			return 15.0F;
		case 4:
			return 19.0F;
		default:
			break;
		}

		value = static_cast<float>(z);

		if (value < 15.0F) {
			return value + value;
		}

		break;
	default:
		return static_cast<float>(y) * 0.66F * static_cast<float>(z);
	}

	return value * 1.5F;
}

float Constants::contactBiasRate() noexcept
{
	return -std::log(contactReferenceRetention()) * static_cast<float>(kernelStepsPerSec());
}

float Constants::elasticMultiplier(float elasticity) noexcept
{
	if (elasticity < 0.05F) {
		return 0.28F;
	}

	if (elasticity < 0.26F) {
		return 0.42F;
	}

	if (elasticity < 0.51F) {
		return 0.57F;
	}

	if (elasticity < 0.76F) {
		return 0.8F;
	}

	return 1.0F;
}

float Constants::kmsMaxJointForce(float grid1, float grid2) noexcept
{
	const int grid1int = std::max(1, round(grid1));
	const int grid2int = std::max(1, round(grid2));

	const int width = std::max(grid1int, grid2int);
	const int overlap = std::min(grid1int, grid2int);

	float force = width < 7 ? kMaxLegoJointForcesMeasured[static_cast<std::size_t>(width)]
	                        : static_cast<float>(width) * (1.0F / 7.0F) * kMaxLegoJointForcesMeasured[6];

	force *= 0.5F;

	return force * static_cast<float>(overlap) * kLegoForceScale;
}

float Constants::jointK(const Vector3& gridSize, bool ball) noexcept
{
	const Vector3 sortedSize = sortVector3(gridSize);

	const Vector3 clippedSize{
	    std::max(1.0F, sortedSize.x),
	    std::max(1.0F, sortedSize.y),
	    std::max(1.0F, sortedSize.z),
	};

	const float multiplier = jointKMultiplier(clippedSize, ball);

	if (sortedSize.x < 1.0F) {
		return sortedSize.x * multiplier * kJointKScale;
	}

	return multiplier * kJointKScale;
}

float jointK(const Primitive* primitive) noexcept
{
	const Geometry* geometry = primitive->geometry;

	if (geometry == nullptr) {
		return 0.0F;
	}

	return Constants::jointK(geometry->gridSize, geometry->type() == Geometry::Ball);
}

} // namespace v8patch::rbx
