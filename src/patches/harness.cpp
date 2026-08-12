#include <cstdarg>
#include <cstdio>

#include "hook.h"
#include "log.h"
#include "offsets.h"
#include "patch.h"

namespace v8patch {
namespace {

using PrintFn = void(__cdecl*)(void*, int, const char*, ...);

PrintFn g_print = nullptr;

void __cdecl onPrint(void* self, int type, const char* format, ...)
{
	char line[2048];

	va_list args;
	va_start(args, format);
	const int written = std::vsnprintf(line, sizeof(line), format, args);
	va_end(args);

	if (written > 0) {
		log::info("[script] {}", line);
	}

	g_print(self, type, "%s", line);
}

using DemandFn = void(__stdcall*)(int, int);

DemandFn g_demand = nullptr;

void __stdcall onDemand(int, int) {}

} // namespace

bool installHarness(const Target& target)
{
	HookSet hooks(target);

	if (!hooks.install(offsets::StandardOut::kPrint, &onPrint, g_print) ||
	    !hooks.install(offsets::Security::kDemand, &onDemand, g_demand)) {
		return false;
	}

	return true;
}

} // namespace v8patch
