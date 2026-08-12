#include <windows.h>

#include <MinHook.h>

#include "log.h"
#include "patch.h"
#include "proxy.h"
#include "target.h"

namespace {

DWORD WINAPI start(LPVOID)
{
	using namespace v8patch;

	log::open();
	log::info("v8patch " V8PATCH_VERSION);

	proxy::load();

	const auto target = Target::resolve();
	if (!target.has_value()) {
		return 1;
	}

	log::info("host {} at {:#x}", target->version().str(), target->base());

	if (target->version() != kSupported) {
		log::error("unsupported build {}, expected {}", target->version().str(), kSupported.str());
		return 1;
	}

	if (MH_Initialize() != MH_OK) {
		log::error("MinHook failed to initialise");
		return 1;
	}

	applyAll(*target);
	return 0;
}

} // namespace

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID)
{
	switch (reason) {
	case DLL_PROCESS_ATTACH: {
		DisableThreadLibraryCalls(module);
		HANDLE thread = CreateThread(nullptr, 0, &start, nullptr, 0, nullptr);
		if (thread != nullptr) {
			CloseHandle(thread);
		}
		break;
	}
	case DLL_PROCESS_DETACH:
		MH_Uninitialize();
		v8patch::log::close();
		break;
	default:
		break;
	}

	return TRUE;
}
