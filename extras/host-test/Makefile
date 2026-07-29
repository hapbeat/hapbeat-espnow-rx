# ---------------------------------------------------------------------------
# Host regression tests for hapbeat-espnow-rx.
#
#   make          build and run
#   make build    build only
#   make clean    remove build artefacts
#   make run ARGS=opus     run only tests whose name contains "opus"
#
# Deliberately plain g++: these tests double as the proof that src/protocol/
# and src/codec/ contain no Arduino.h, no esp_now.h and no millis(). If the
# library ever grows an ESP32 dependency in those directories, this build
# stops compiling — which is the point.
#
# Opus is NOT enabled here (no -DHAPBEAT_ESPNOW_RX_OPUS): the Opus wire-format
# tests inject a fake backend through StreamDecoder::setOpusBackend(), so the
# format is verified without libopus on the machine.
#
# Object files land in this directory rather than a build/ subdirectory: MinGW
# make 3.82 mishandles order-only prerequisites on pattern rules ("Circular
# ... dependency dropped") and then compiles into a directory it never created.
# ---------------------------------------------------------------------------

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -O1 -g
SRC      := ../../src
INCLUDES := -I$(SRC) -I.

BIN := host-test

# $(sort) also de-duplicates, in case the mode table lives under protocol/.
LIB_SRCS := $(sort $(wildcard $(SRC)/protocol/*.cpp) \
                   $(wildcard $(SRC)/codec/*.cpp) \
                   $(wildcard $(SRC)/HapbeatStreamModes.cpp))

TEST_SRCS := $(sort $(wildcard test_*.cpp))

ALL_SRCS := $(LIB_SRCS) $(TEST_SRCS)
OBJS     := $(notdir $(ALL_SRCS:.cpp=.o))

# Source basenames are unique across these directories, so a flat object layout
# is safe and keeps the rules simple.
vpath %.cpp . $(SRC) $(SRC)/protocol $(SRC)/codec

.PHONY: all build run clean

all: run

build: $(BIN)

run: $(BIN)
	@./$(BIN) $(ARGS)

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

clean:
	rm -f *.o *.d $(BIN) $(BIN).exe

-include $(OBJS:.o=.d)
