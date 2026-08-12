#include "target.h"

#include <windows.h>

#include <cstddef>
#include <format>
#include <string_view>
#include <vector>

#include "log.h"

#pragma comment(lib, "version.lib")

namespace v8patch {
namespace {

std::optional<Version> readVersion(HMODULE module)
{
	wchar_t path[MAX_PATH]{};
	if (GetModuleFileNameW(module, path, MAX_PATH) == 0) {
		return std::nullopt;
	}

	DWORD ignored = 0;
	const DWORD size = GetFileVersionInfoSizeW(path, &ignored);
	if (size == 0) {
		return std::nullopt;
	}

	std::vector<std::byte> buffer(size);
	if (!GetFileVersionInfoW(path, 0, size, buffer.data())) {
		return std::nullopt;
	}

	VS_FIXEDFILEINFO* info = nullptr;
	UINT length = 0;
	if (!VerQueryValueW(buffer.data(), L"\\", reinterpret_cast<void**>(&info), &length) || info == nullptr) {
		return std::nullopt;
	}

	return Version{
	    static_cast<std::uint16_t>(HIWORD(info->dwFileVersionMS)),
	    static_cast<std::uint16_t>(LOWORD(info->dwFileVersionMS)),
	    static_cast<std::uint16_t>(HIWORD(info->dwFileVersionLS)),
	    static_cast<std::uint16_t>(LOWORD(info->dwFileVersionLS)),
	};
}

int hexDigit(char c) noexcept
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	return -1;
}

} // namespace

std::string Version::str() const
{
	return std::format("{}.{}.{}.{}", major, minor, build, revision);
}

std::optional<Target> Target::resolve()
{
	HMODULE module = GetModuleHandleW(nullptr);
	if (module == nullptr) {
		log::error("the host module could not be resolved");
		return std::nullopt;
	}

	const auto version = readVersion(module);
	if (!version.has_value()) {
		log::error("the host carries no version resource");
		return std::nullopt;
	}

	return Target{reinterpret_cast<std::uintptr_t>(module), *version};
}

bool Target::matches(const offsets::Site& site) const noexcept
{
	const auto* bytes = reinterpret_cast<const std::uint8_t*>(base_ + site.offset);
	const std::string_view pattern = site.signature;

	std::size_t index = 0;
	for (std::size_t i = 0; i < pattern.size();) {
		if (pattern[i] == ' ') {
			++i;
			continue;
		}
		if (i + 1 >= pattern.size()) {
			return false;
		}

		if (pattern[i] == '?') {
			i += 2;
			++index;
			continue;
		}

		const int hi = hexDigit(pattern[i]);
		const int lo = hexDigit(pattern[i + 1]);
		if (hi < 0 || lo < 0) {
			return false;
		}

		if (bytes[index] != static_cast<std::uint8_t>(hi << 4 | lo)) {
			return false;
		}

		i += 2;
		++index;
	}

	return true;
}

} // namespace v8patch
