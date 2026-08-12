#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "offsets.h"

namespace v8patch {

struct Version
{
	std::uint16_t major{};
	std::uint16_t minor{};
	std::uint16_t build{};
	std::uint16_t revision{};

	[[nodiscard]] std::string str() const;
	friend bool operator==(const Version&, const Version&) = default;
};

inline constexpr Version kSupported{0, 3, 676, 0};

class Target
{
public:
	static std::optional<Target> resolve();

	[[nodiscard]] std::uintptr_t base() const noexcept { return base_; }
	[[nodiscard]] Version version() const noexcept { return version_; }

	[[nodiscard]] void* rva(std::uintptr_t offset) const noexcept { return reinterpret_cast<void*>(base_ + offset); }

	[[nodiscard]] bool matches(const offsets::Site& site) const noexcept;

private:
	Target(std::uintptr_t base, Version version) noexcept : base_(base), version_(version) {}

	std::uintptr_t base_{};
	Version version_{};
};

} // namespace v8patch
