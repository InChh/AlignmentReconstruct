#include "test_framework.h"

#include <exception>
#include <iostream>

int main() {
    std::size_t failures = 0;
    std::size_t skipped = 0;
    for (const auto& test : test_support::registry()) {
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

    std::cout << "Executed " << test_support::registry().size()
              << " tests, failures: " << failures
              << ", skipped: " << skipped << '\n';
    return failures == 0 ? 0 : 1;
}
