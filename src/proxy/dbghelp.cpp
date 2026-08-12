#include "proxy.h"

#include <windows.h>

#include <atomic>
#include <string>

#include "log.h"

#pragma comment(linker, "/EXPORT:MiniDumpWriteDump=_MiniDumpWriteDump@28")

namespace v8patch::proxy {
namespace {

using MiniDumpWriteDumpFn = BOOL(WINAPI*)(HANDLE, DWORD, HANDLE, DWORD, void*, void*, void*);

std::atomic<MiniDumpWriteDumpFn> g_forward{nullptr};

HMODULE loadSystemCopy()
{
	wchar_t directory[MAX_PATH]{};
	const UINT length = GetSystemDirectoryW(directory, MAX_PATH);
	if (length == 0 || length >= MAX_PATH) {
		return nullptr;
	}

	std::wstring path(directory, length);
	path += L"\\dbghelp.dll";

	return LoadLibraryW(path.c_str());
}

} // namespace

MiniDumpWriteDumpFn forward()
{
	MiniDumpWriteDumpFn cached = g_forward.load(std::memory_order_acquire);
	if (cached != nullptr) {
		return cached;
	}

	HMODULE real = loadSystemCopy();
	if (real == nullptr) {
		return nullptr;
	}

	auto resolved = reinterpret_cast<MiniDumpWriteDumpFn>(GetProcAddress(real, "MiniDumpWriteDump"));
	if (resolved == nullptr) {
		FreeLibrary(real);
		return nullptr;
	}

	MiniDumpWriteDumpFn expected = nullptr;
	if (!g_forward.compare_exchange_strong(expected, resolved, std::memory_order_acq_rel)) {
		FreeLibrary(real);
		return expected;
	}

	return resolved;
}

bool load()
{
	if (forward() == nullptr) {
		log::error("could not forward MiniDumpWriteDump to the system dbghelp.dll");
		return false;
	}

	return true;
}

} // namespace v8patch::proxy

extern "C" BOOL WINAPI MiniDumpWriteDump(
    HANDLE process,
    DWORD processId,
    HANDLE file,
    DWORD type,
    void* exceptionParam,
    void* userStreamParam,
    void* callbackParam
)
{
	const auto real = v8patch::proxy::forward();
	if (real == nullptr) {
		SetLastError(ERROR_PROC_NOT_FOUND);
		return FALSE;
	}

	return real(process, processId, file, type, exceptionParam, userStreamParam, callbackParam);
}
