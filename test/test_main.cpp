// The single doctest implementation TU. Every other test_*.cpp just includes
// "doctest.h" and adds TEST_CASEs; this file provides main().
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

// A trivial smoke test proving the harness itself works (Phase 0).
TEST_CASE("harness works") {
    CHECK(1 + 1 == 2);
}
