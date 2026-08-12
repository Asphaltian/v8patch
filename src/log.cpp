#include "log.h"

#include <windows.h>

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

#include "paths.h"

namespace v8patch::log {
namespace {

std::mutex g_mutex;
std::ofstream g_file;
DWORD g_pid = 0;

std::string_view tag(Level level) noexcept
{
	switch (level) {
	case Level::Info:
		return "info";
	case Level::Warn:
		return "warn";
	case Level::Error:
		return "error";
	}
	return "?";
}

} // namespace

void open()
{
	const std::scoped_lock lock(g_mutex);
	g_file.open(beside(L"v8patch.log"), std::ios::out | std::ios::app);
	g_pid = GetCurrentProcessId();
}

void close()
{
	const std::scoped_lock lock(g_mutex);
	if (g_file.is_open()) {
		g_file.flush();
		g_file.close();
	}
}

void write(Level level, std::string_view message)
{
	const auto now = std::chrono::current_zone()->to_local(std::chrono::system_clock::now());
	const auto stamp = std::chrono::floor<std::chrono::milliseconds>(now);
	const auto line = std::format("[{:%H:%M:%S}] [{}] {}\n", stamp, tag(level), message);

	const std::scoped_lock lock(g_mutex);
	if (g_file.is_open()) {
		g_file << line;
		g_file.flush();
	}

	OutputDebugStringA(line.c_str());
}

} // namespace v8patch::log
