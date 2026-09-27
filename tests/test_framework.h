// zeliboba - minimal self test framework.
//
// Usage:
//
//     #include "test_framework.h"
//
//     ZLB_TEST(arm_add_immediate) {
//         ArmCore cpu(bus);
//         ...
//         ZLB_EXPECT_EQ(cpu.read_reg(0), 0x1234u);
//         ZLB_EXPECT_TRUE(condition);
//     }
#pragma once

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace zlb::test {

struct TestCase {
    std::string name;
    std::string file;
    std::function<void()> body;
};

std::vector<TestCase>& registry();

int failures();
void report_failure(const char* file, int line, const std::string& message);

struct Registrar {
    Registrar(const char* name, const char* file, std::function<void()> body);
};

}  // namespace zlb::test

#define ZLB_TEST(name)                                                             \
    static void zlb_test_##name();                                                 \
    static ::zlb::test::Registrar zlb_registrar_##name(#name, __FILE__, zlb_test_##name); \
    static void zlb_test_##name()

#define ZLB_EXPECT_TRUE(condition)                                                 \
    do {                                                                           \
        if (!(condition)) ::zlb::test::report_failure(__FILE__, __LINE__, "expected true: " #condition); \
    } while (0)

#define ZLB_EXPECT_FALSE(condition)                                                \
    do {                                                                           \
        if (condition) ::zlb::test::report_failure(__FILE__, __LINE__, "expected false: " #condition); \
    } while (0)

#define ZLB_EXPECT_EQ(a, b)                                                        \
    do {                                                                           \
        auto zlb_a = (a);                                                          \
        auto zlb_b = (b);                                                          \
        if (!(zlb_a == zlb_b)) {                                                   \
            char zlb_buffer[512];                                                  \
            std::snprintf(zlb_buffer, sizeof(zlb_buffer),                          \
                          "expected %s == %s (0x%llX vs 0x%llX)", #a, #b,          \
                          (unsigned long long)zlb_a, (unsigned long long)zlb_b);   \
            ::zlb::test::report_failure(__FILE__, __LINE__, zlb_buffer);           \
        }                                                                          \
    } while (0)

#define ZLB_EXPECT_NE(a, b)                                                        \
    do {                                                                           \
        auto zlb_a = (a);                                                          \
        auto zlb_b = (b);                                                          \
        if (zlb_a == zlb_b) ::zlb::test::report_failure(__FILE__, __LINE__, "expected " #a " != " #b); \
    } while (0)

#define ZLB_EXPECT_NEAR(a, b, eps)                                                 \
    do {                                                                           \
        double zlb_a = (double)(a);                                                \
        double zlb_b = (double)(b);                                                \
        if (!((zlb_a - zlb_b) < (eps) && (zlb_b - zlb_a) < (eps)))                 \
            ::zlb::test::report_failure(__FILE__, __LINE__, "expected " #a " ~= " #b); \
    } while (0)

#define ZLB_FAIL(message) ::zlb::test::report_failure(__FILE__, __LINE__, message)
