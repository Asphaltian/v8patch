#include "paths.h"

#include <windows.h>

namespace v8patch {

std::wstring beside(std::wstring_view name)
{
	HMODULE self = nullptr;
	GetModuleHandleExW(
	    GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
	    reinterpret_cast<LPCWSTR>(&beside),
	    &self
	);

	wchar_t path[MAX_PATH]{};
	const DWORD length = GetModuleFileNameW(self, path, MAX_PATH);

	std::wstring result(path, length);
	const auto slash = result.find_last_of(L'\\');
	result.resize(slash == std::wstring::npos ? 0 : slash + 1);
	result += name;

	return result;
}

} // namespace v8patch
