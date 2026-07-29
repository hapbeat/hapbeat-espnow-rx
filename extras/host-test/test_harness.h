#pragma once
// ---------------------------------------------------------------------------
// test_harness.h — the whole test framework. No external dependencies: this
// suite has to build with nothing but g++ on a bare PC, because its job is to
// prove that src/protocol/ and src/codec/ are free of Arduino/ESP-NOW.
//
// Usage:
//   HT_TEST(my_case) { HT_CHECK_EQ(got, want); }
// Registration happens through a namespace-scope constructor, so tests in
// different translation units self-register with no central list to update.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace ht {

typedef void (*TestFn)();

// Defined in test_main.cpp. registerTest() is called from static constructors,
// so its storage lives in a function-local static (no static-init-order trap).
int  registerTest(const char* name, TestFn fn);
void fail(const char* file, int line, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));
void noteCheck();

} // namespace ht

// --- test definition --------------------------------------------------------
#define HT_TEST(name)                                                          \
    static void name();                                                        \
    namespace {                                                                \
    struct name##_Registrar {                                                  \
        name##_Registrar() { ht::registerTest(#name, name); }                  \
    } name##_registrar_instance;                                               \
    }                                                                          \
    static void name()

// --- assertions -------------------------------------------------------------
// Every failure prints expected vs actual: a bare "assertion failed" is useless
// when the thing under test is a byte offset in a wire format.

#define HT_CHECK(cond)                                                         \
    do {                                                                       \
        ht::noteCheck();                                                       \
        if (!(cond)) ht::fail(__FILE__, __LINE__, "expected true: %s", #cond);  \
    } while (0)

#define HT_CHECKF(cond, ...)                                                   \
    do {                                                                       \
        ht::noteCheck();                                                       \
        if (!(cond)) ht::fail(__FILE__, __LINE__, __VA_ARGS__);                 \
    } while (0)

// `long` and %ld rather than long long / %lld: MinGW's msvcrt printf does not
// support %lld, and this suite has to run on the same Windows toolchain the
// firmware is built with. Every counter compared here is small.
#define HT_CHECK_EQ(actual, expected)                                          \
    do {                                                                       \
        ht::noteCheck();                                                       \
        long ht_a_ = (long)(actual);                                           \
        long ht_e_ = (long)(expected);                                         \
        if (ht_a_ != ht_e_)                                                    \
            ht::fail(__FILE__, __LINE__, "%s: expected %ld, actual %ld",       \
                     #actual, ht_e_, ht_a_);                                   \
    } while (0)

// Same, plus a caller-supplied explanation of what the mismatch means.
#define HT_CHECK_EQ_WHY(actual, expected, why)                                 \
    do {                                                                       \
        ht::noteCheck();                                                       \
        long ht_a_ = (long)(actual);                                           \
        long ht_e_ = (long)(expected);                                         \
        if (ht_a_ != ht_e_)                                                    \
            ht::fail(__FILE__, __LINE__,                                       \
                     "%s: expected %ld, actual %ld  <- %s",                    \
                     #actual, ht_e_, ht_a_, why);                              \
    } while (0)

// Compare int16 arrays element by element; reports the first differing index.
#define HT_CHECK_PCM(actual, expected, count, label)                           \
    do {                                                                       \
        ht::noteCheck();                                                       \
        const int16_t* ht_ap_ = (actual);                                      \
        const int16_t* ht_ep_ = (expected);                                    \
        int ht_n_ = (int)(count);                                              \
        int ht_bad_ = -1;                                                      \
        for (int ht_i_ = 0; ht_i_ < ht_n_; ht_i_++)                            \
            if (ht_ap_[ht_i_] != ht_ep_[ht_i_]) { ht_bad_ = ht_i_; break; }    \
        if (ht_bad_ >= 0)                                                      \
            ht::fail(__FILE__, __LINE__,                                       \
                     "%s: sample[%d] expected %d, actual %d (%d samples)",     \
                     label, ht_bad_, (int)ht_ep_[ht_bad_],                     \
                     (int)ht_ap_[ht_bad_], ht_n_);                             \
    } while (0)

#define HT_CHECK_BYTES(actual, expected, count, label)                         \
    do {                                                                       \
        ht::noteCheck();                                                       \
        const uint8_t* ht_ab_ = (const uint8_t*)(actual);                      \
        const uint8_t* ht_eb_ = (const uint8_t*)(expected);                    \
        int ht_n_ = (int)(count);                                              \
        int ht_bad_ = -1;                                                      \
        for (int ht_i_ = 0; ht_i_ < ht_n_; ht_i_++)                            \
            if (ht_ab_[ht_i_] != ht_eb_[ht_i_]) { ht_bad_ = ht_i_; break; }    \
        if (ht_bad_ >= 0)                                                      \
            ht::fail(__FILE__, __LINE__,                                       \
                     "%s: byte[%d] expected 0x%02X, actual 0x%02X (%d bytes)", \
                     label, ht_bad_, ht_eb_[ht_bad_], ht_ab_[ht_bad_], ht_n_); \
    } while (0)
