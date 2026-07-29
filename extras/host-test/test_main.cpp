// ---------------------------------------------------------------------------
// test_main.cpp — harness storage + runner.
//
// Exit status is what CI reads: 0 = all green, 1 = at least one failure.
// ---------------------------------------------------------------------------

#include "test_harness.h"

#include <stdarg.h>
#include <stdlib.h>

namespace ht {
namespace {

const int MAX_TESTS = 128;

struct Entry {
    const char* name;
    TestFn      fn;
};

// Function-local statics: tests register from static constructors in other
// translation units, which may run before this file's own globals would.
Entry* registry() {
    static Entry table[MAX_TESTS];
    return table;
}

int& registryCount() {
    static int n = 0;
    return n;
}

int& totalChecks() {
    static int n = 0;
    return n;
}

int& totalFailures() {
    static int n = 0;
    return n;
}

int& currentTestFailures() {
    static int n = 0;
    return n;
}

const char*& currentTestName() {
    static const char* s = "(none)";
    return s;
}

} // namespace

int registerTest(const char* name, TestFn fn) {
    int& n = registryCount();
    if (n >= MAX_TESTS) {
        fprintf(stderr, "test registry full (MAX_TESTS=%d)\n", MAX_TESTS);
        abort();
    }
    registry()[n].name = name;
    registry()[n].fn   = fn;
    return n++;
}

void noteCheck() { totalChecks()++; }

void fail(const char* file, int line, const char* fmt, ...) {
    totalFailures()++;
    currentTestFailures()++;
    fprintf(stderr, "  FAIL %s:%d  [%s]\n         ", file, line, currentTestName());
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

} // namespace ht

int main(int argc, char** argv) {
    using namespace ht;

    const char* filter = (argc > 1) ? argv[1] : NULL;

    int run = 0, failed = 0;
    for (int i = 0; i < registryCount(); i++) {
        const Entry& e = registry()[i];
        if (filter && !strstr(e.name, filter)) continue;

        currentTestName()     = e.name;
        currentTestFailures() = 0;
        run++;

        e.fn();

        if (currentTestFailures() == 0) {
            printf("  ok   %s\n", e.name);
        } else {
            printf("  FAIL %s (%d failed checks)\n", e.name, currentTestFailures());
            failed++;
        }
    }

    printf("\n%d tests run, %d failed, %d checks\n", run, failed, totalChecks());
    if (run == 0) {
        fprintf(stderr, "no tests matched — did the link drop a test object?\n");
        return 1;
    }
    return failed == 0 ? 0 : 1;
}
