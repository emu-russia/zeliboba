#include "test_framework.h"

#include <cstring>

namespace zlb::test {

std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

static int& failure_counter() {
    static int count = 0;
    return count;
}

int failures() { return failure_counter(); }

void report_failure(const char* file, int line, const std::string& message) {
    ++failure_counter();
    std::printf("  FAIL %s:%d: %s\n", file, line, message.c_str());
}

Registrar::Registrar(const char* name, const char* file, std::function<void()> body) {
    registry().push_back(TestCase{name, file, std::move(body)});
}

}  // namespace zlb::test

int main(int argc, char** argv) {
    auto& tests = zlb::test::registry();
    const char* filter = argc > 1 ? argv[1] : nullptr;

    int run = 0;
    int failed_before = 0;
    int failed_cases = 0;

    for (auto& test : tests) {
        if (filter && test.name.find(filter) == std::string::npos) continue;
        ++run;
        failed_before = zlb::test::failures();
        std::printf("[ RUN  ] %s\n", test.name.c_str());
        test.body();
        if (zlb::test::failures() != failed_before) {
            ++failed_cases;
            std::printf("[ FAIL ] %s\n", test.name.c_str());
        } else {
            std::printf("[  OK  ] %s\n", test.name.c_str());
        }
    }

    std::printf("\n%d test(s) run, %d case(s) failed, %d assertion failure(s)\n", run, failed_cases,
                zlb::test::failures());
    return zlb::test::failures() == 0 ? 0 : 1;
}
