#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <string>

namespace voxel4d::test {

class TestContext {
   public:
    void expect(const bool condition, const std::string& message) {
        if (!condition) {
            std::cerr << "FAILED: " << message << '\n';
            ++failures_;
        }
    }

    void expect_near(const float actual, const float expected, const float tolerance,
                     const std::string& message) {
        expect(std::fabs(actual - expected) <= tolerance,
               message + " (actual=" + std::to_string(actual) +
                   ", expected=" + std::to_string(expected) + ")");
    }

    template <typename ExceptionType>
    void expect_throws(const std::function<void()>& callable, const std::string& message) {
        try {
            callable();
        } catch (const ExceptionType&) {
            return;
        } catch (const std::exception& error) {
            expect(false, message + " (unexpected exception: " + error.what() + ")");
            return;
        }
        expect(false, message + " (no exception thrown)");
    }

    [[nodiscard]] int failures() const {
        return failures_;
    }

   private:
    int failures_{0};
};

}  // namespace voxel4d::test
