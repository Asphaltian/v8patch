#include "patch.h"

#include <iterator>

#include "config.h"
#include "log.h"

namespace v8patch {
namespace {

constexpr Patch kPatches[]{
    {"framerate", true, &installFramerate},
    {"physics", true, &installPhysics},
    {"harness", false, &installHarness},
};

} // namespace

void applyAll(const Target& target)
{
	std::size_t applied = 0;

	for (const Patch& patch : kPatches) {
		if (!config::boolean(patch.name, "enabled", patch.enabledByDefault)) {
			log::info("{} disabled", patch.name);
			continue;
		}

		if (patch.install(target)) {
			log::info("{} applied", patch.name);
			++applied;
		} else {
			log::error("{} failed", patch.name);
		}
	}

	log::info("{} of {} patches active", applied, std::size(kPatches));
}

} // namespace v8patch
