#pragma once

#include <string_view>

#include "target.h"

namespace v8patch {

struct Patch
{
	std::string_view name;
	bool enabledByDefault;
	bool (*install)(const Target& target);
};

bool installFramerate(const Target& target);
bool installPhysics(const Target& target);
bool installHarness(const Target& target);
bool ownsWorldStep();

void applyAll(const Target& target);

} // namespace v8patch
