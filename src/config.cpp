#include "config.h"

#include <windows.h>

#include <string>

#include "log.h"
#include "paths.h"

namespace v8patch::config {
namespace {

std::wstring widen(std::string_view text)
{
	if (text.empty()) {
		return {};
	}

	const int size = MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
	std::wstring result(static_cast<std::size_t>(size), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), result.data(), size);

	return result;
}

const std::wstring& file()
{
	static const std::wstring path = [] {
		std::wstring found = beside(L"v8patch.ini");

		if (GetFileAttributesW(found.c_str()) == INVALID_FILE_ATTRIBUTES) {
			log::warn("no v8patch.ini beside the dll, every patch falls back to its default");
			found.clear();
		}

		return found;
	}();

	return path;
}

} // namespace

bool boolean(std::string_view section, std::string_view key, bool fallback)
{
	return integer(section, key, fallback ? 1 : 0) != 0;
}

int integer(std::string_view section, std::string_view key, int fallback)
{
	const std::wstring& path = file();
	if (path.empty()) {
		return fallback;
	}

	return static_cast<int>(GetPrivateProfileIntW(widen(section).c_str(), widen(key).c_str(), fallback, path.c_str()));
}

} // namespace v8patch::config
