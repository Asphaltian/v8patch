#pragma once

#include <cmath>
#include <cstdio>

namespace v8patch::tests {

inline int g_checked = 0;
inline int g_failed = 0;

inline void check(bool ok, const char* what)
{
	++g_checked;

	if (!ok) {
		++g_failed;
		std::printf("FAIL %s\n", what);
	}
}

inline void same(int got, int want, const char* what)
{
	++g_checked;

	if (got != want) {
		++g_failed;
		std::printf("FAIL %s: got %d want %d\n", what, got, want);
	}
}

inline void near(float got, float want, float tolerance, const char* what)
{
	++g_checked;

	if (!(std::fabs(got - want) <= tolerance)) {
		++g_failed;
		std::printf("FAIL %s: got %.6g want %.6g\n", what, got, want);
	}
}

} // namespace v8patch::tests
