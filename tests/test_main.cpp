#include "test_framework.h"

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    const std::string filter = argc > 1 ? argv[1] : "";
    std::size_t executed = 0;
    std::size_t failures = 0;
    std::size_t skipped = 0;
    for (const auto& test : test_support::registry()) {
        if (!filter.empty() && test.name.find(filter) == std::string::npos) {
            continue;
        }
        ++executed;
        try {
            test.function();
            std::cout << "[PASS] " << test.name << '\n';
        }
        catch (const test_support::SkippedTest& reason) {
            ++skipped;
            std::cout << "[SKIP] " << test.name << ": " << reason.what() << '\n';
        }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "[FAIL] " << test.name << ": " << error.what() << '\n';
        }
    }

    std::cout << "Executed " << executed
              << " tests, failures: " << failures
              << ", skipped: " << skipped << '\n';
    return failures == 0 ? 0 : 1;
}
