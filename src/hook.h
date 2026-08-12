#pragma once

#include <cstddef>

#include "offsets.h"
#include "target.h"

namespace v8patch {

class HookSet
{
public:
	explicit HookSet(const Target& target) noexcept : target_(&target) {}

	template <typename Fn>
	bool install(const offsets::Site& site, Fn detour, Fn& original)
	{
		return attach(site, reinterpret_cast<void*>(detour), reinterpret_cast<void**>(&original));
	}

	[[nodiscard]] std::size_t count() const noexcept { return count_; }

private:
	bool attach(const offsets::Site& site, void* detour, void** original);

	const Target* target_;
	std::size_t count_{};
};

} // namespace v8patch
