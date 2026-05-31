UNITY   ?= /tmp/Unity-2.5.2/src
CXX     := g++
CXXFLAGS := -std=c++17 -I ../test/mocks -I ../src -I $(UNITY)

# All src/*.cpp except the real RaceBoxBle (replaced by the stub below).
SRC_FILES := $(filter-out ../src/RaceBoxBle.cpp, $(wildcard ../src/*.cpp))

# Stub replacing RaceBoxBle.cpp for native builds.
STUB_FILES := ../test/mocks/RaceBoxBle_stub.cpp

.PHONY: all test download-unity

all: test

download-unity:
	@if [ ! -d $(UNITY) ]; then \
	  echo "Downloading Unity 2.5.2..."; \
	  curl -sL https://github.com/ThrowTheSwitch/Unity/archive/refs/tags/v2.5.2.tar.gz | tar -xz -C /tmp; \
	fi

test: download-unity
	@FAIL=0; \
	for t in test_*.cpp; do \
	  name=$$(basename $$t .cpp); \
	  $(CXX) $(CXXFLAGS) $(SRC_FILES) $(STUB_FILES) $(UNITY)/unity.c $$t -o /tmp/$$name 2>&1; \
	  echo "=== $$name ==="; \
	  /tmp/$$name || FAIL=$$((FAIL + 1)); \
	done; \
	exit $$FAIL
