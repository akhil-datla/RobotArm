# RobotArm host test Makefile — dependency-free.
#
# Requires only: a C++ compiler (g++ or clang++) and make. No CMake, no Python,
# no package managers, no network. The vendored doctest.h (test/doctest.h) is the
# only test framework. Run `make test` constantly during TDD.

CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2 -g
INCLUDES := -I../src -I../src/core -I../src/hal -I. -Ifakes

# Host-buildable production sources: the pure core and the public component layer.
# NOTHING under src/arduino/ is compiled here — those files include Adafruit /
# Wire / Arduino headers and only build inside the Arduino toolchain.
CORE_SRCS := $(wildcard ../src/core/*.cpp)
PUB_SRCS  := $(wildcard ../src/*.cpp)
SRCS      := $(CORE_SRCS) $(PUB_SRCS)

# Test translation units.
TEST_SRCS := $(wildcard test_*.cpp)

BIN := run_tests

# ---- primary TDD command -------------------------------------------------
.PHONY: all test check-core-purity check-includes clean

# Default goal: prove the core is Arduino-free, prove the umbrella resolves under
# the Arduino include layout, then run the suite.
all: check-core-purity check-includes test

test: $(BIN)
	./$(BIN)

$(BIN): $(TEST_SRCS) $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TEST_SRCS) $(SRCS) -o $(BIN)

# ---- mechanical "pure core" enforcement ----------------------------------
# Fails if any file under src/core/ references an Arduino-only symbol. This is
# what guarantees the math is host-testable and portable to any C++ compiler.
FORBIDDEN := Arduino\.h|Wire\.h|Adafruit_|Servo\.h|Serial|String|\bmillis\b|\bdelay\b|analogWrite|digitalWrite|new |\bmalloc\b

check-core-purity:
	@echo "Checking src/core/ for forbidden Arduino symbols..."
	@if grep -rnE '$(FORBIDDEN)' ../src/core/ ; then \
		echo "FAIL: forbidden Arduino symbol found in src/core/ (see above)."; \
		exit 1; \
	else \
		echo "OK: src/core/ is Arduino-free."; \
	fi

# ---- Arduino include-layout guard ----------------------------------------
# The Arduino IDE adds ONLY src/ to the include path (recursive-library layout),
# not its subfolders. So the umbrella and every public header it pulls in must
# reference core/ and hal/ headers relative to src/. This target syntax-checks
# <RobotArm.h> with -Isrc alone, catching any bare cross-directory include that
# would break `#include <RobotArm.h>` on real hardware. (ARDUINO is left
# undefined so the check needs no Adafruit/Wire headers; the #ifdef ARDUINO block
# is exercised by the stub-compile in CI/dev notes.)
check-includes:
	@echo "Checking <RobotArm.h> resolves with only src/ on the include path..."
	@echo '#include <RobotArm.h>' | $(CXX) -std=c++17 -I../src -x c++ -fsyntax-only - \
		&& echo "OK: umbrella header resolves under the Arduino include layout." \
		|| (echo "FAIL: a header uses a bare cross-directory include (prefix it with core/ or hal/)."; exit 1)

clean:
	rm -f $(BIN)
