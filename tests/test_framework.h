#pragma once

#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace test_support {

class SkippedTest final : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct TestCase {
    std::string name;
    std::function<void()> function;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

[[noreturn]] inline void skip(const std::string& reason) {
    throw SkippedTest(reason);
}

class Registrar {
public:
    Registrar(std::string name, std::function<void()> function) {
        registry().push_back({std::move(name), std::move(function)});
    }
};

[[noreturn]] inline void fail(const char* expression, const char* file, int line, const std::string& detail = {}) {
    std::ostringstream stream;
    stream << file << ':' << line << ": expectation failed: " << expression;
    if (!detail.empty()) {
        stream << " (" << detail << ')';
    }
    throw std::runtime_error(stream.str());
}

inline void expect_near(double actual, double expected, double tolerance, const char* file, int line) {
    if (!std::isfinite(actual) || !std::isfinite(expected) || std::abs(actual - expected) > tolerance) {
        std::ostringstream detail;
        detail << std::setprecision(17);
        detail << "actual=" << actual << ", expected=" << expected << ", tolerance=" << tolerance;
        fail("near", file, line, detail.str());
    }
}

} // namespace test_support

#define AR_TEST(name) \
    static void name(); \
    static const test_support::Registrar name##_registrar(#name, &name); \
    static void name()

#define AR_EXPECT(expression) \
    do { if (!(expression)) test_support::fail(#expression, __FILE__, __LINE__); } while (false)

#define AR_EXPECT_NEAR(actual, expected, tolerance) \
    test_support::expect_near((actual), (expected), (tolerance), __FILE__, __LINE__)
