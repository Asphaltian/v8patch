#include "hook.h"

#include <MinHook.h>

#include <cstdint>

#include "log.h"

namespace v8patch {

bool HookSet::attach(const offsets::Site& site, void* detour, void** original)
{
	if (!target_->matches(site)) {
		log::error("{} signature mismatch at +{:#x}", site.name, site.offset);
		return false;
	}

	void* address = target_->rva(site.offset);

	if (MH_CreateHook(address, detour, original) != MH_OK) {
		log::error("{} could not be hooked", site.name);
		return false;
	}

	if (MH_EnableHook(address) != MH_OK) {
		log::error("{} could not be enabled", site.name);
		return false;
	}

	if (*static_cast<const std::uint8_t*>(address) != 0xE9) {
		log::error("{} was not detoured", site.name);
		return false;
	}

	++count_;
	return true;
}

} // namespace v8patch
