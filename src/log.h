#pragma once

#include <format>
#include <string_view>

namespace v8patch::log {

enum class Level
{
	Info,
	Warn,
	Error,
};

void open();
void close();
void write(Level level, std::string_view message);

template <typename... Args>
void info(std::format_string<Args...> fmt, Args&&... args)
{
	write(Level::Info, std::format(fmt, std::forward<Args>(args)...));
}

template <typename... Args>
void warn(std::format_string<Args...> fmt, Args&&... args)
{
	write(Level::Warn, std::format(fmt, std::forward<Args>(args)...));
}

template <typename... Args>
void error(std::format_string<Args...> fmt, Args&&... args)
{
	write(Level::Error, std::format(fmt, std::forward<Args>(args)...));
}

} // namespace v8patch::log
