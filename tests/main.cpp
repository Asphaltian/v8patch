#include "check.h"

#include <cstring>
#include <string_view>

namespace v8patch::tests {

void runClumps();
void runConstants();
void runContactlaw();
void runContacts();
void runFrame();
void runHashing();
void runStepping();
void runWinding();

namespace {

struct Suite
{
	std::string_view name;
	void (*run)();
};

constexpr Suite kSuites[] = {
    {"clumps", &runClumps},
    {"constants", &runConstants},
    {"contactlaw", &runContactlaw},
    {"contacts", &runContacts},
    {"frame", &runFrame},
    {"hashing", &runHashing},
    {"stepping", &runStepping},
    {"winding", &runWinding},
};

int report(std::string_view name)
{
	if (g_failed == 0) {
		std::printf("%.*s: %d checks passed\n", static_cast<int>(name.size()), name.data(), g_checked);
	} else {
		std::printf("%.*s: %d of %d checks failed\n", static_cast<int>(name.size()), name.data(), g_failed, g_checked);
	}

	return g_failed == 0 ? 0 : 1;
}

} // namespace
} // namespace v8patch::tests

int main(int argc, char** argv)
{
	using namespace v8patch::tests;

	if (argc > 1) {
		for (const Suite& suite : kSuites) {
			if (suite.name == argv[1]) {
				suite.run();
				return report(suite.name);
			}
		}

		std::printf("unknown suite %s\n", argv[1]);
		return 2;
	}

	for (const Suite& suite : kSuites) {
		suite.run();
	}

	return report("all");
}
