#pragma once

#include <string_view>

namespace v8patch::config {

bool boolean(std::string_view section, std::string_view key, bool fallback);
int integer(std::string_view section, std::string_view key, int fallback);

} // namespace v8patch::config
