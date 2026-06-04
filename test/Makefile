# On Windows, force Git Bash (sh.exe) so that bash-style loops and /tmp work.
ifeq ($(OS),Windows_NT)
  SHELL := sh
endif

UNITY_VERSION := 2.5.2
UNITY_DIR     := .unity
UNITY         ?= $(UNITY_DIR)/Unity-$(UNITY_VERSION)/src
UNITY_C       := $(UNITY)/unity.c

CXX      := g++
CXXFLAGS := -std=c++17 -I ../test/mocks -I ../src -I $(UNITY)

# All src/*.cpp except the real RaceBoxBle (replaced by the stub below).
SRC_FILES  := $(filter-out ../src/RaceBoxBle.cpp, $(wildcard ../src/*.cpp))
STUB_FILES := ../test/mocks/RaceBoxBle_stub.cpp

UNITY_URL := https://github.com/ThrowTheSwitch/Unity/archive/refs/tags/v$(UNITY_VERSION).tar.gz

# ── OS-specific download command ──────────────────────────────────────────────
ifeq ($(OS),Windows_NT)
  MKDIR_P  = if not exist "$(UNITY_DIR)" mkdir "$(UNITY_DIR)"
  DOWNLOAD = curl -sL $(UNITY_URL) -o $(UNITY_DIR)\unity.tar.gz && \
             tar -xzf $(UNITY_DIR)\unity.tar.gz -C $(UNITY_DIR)
else
  MKDIR_P  = mkdir -p $(UNITY_DIR)
  DOWNLOAD = curl -sL $(UNITY_URL) | tar -xz -C $(UNITY_DIR)
endif

.PHONY: all test clean

all: test

# ── Download Unity only if unity.c is missing (make dependency) ───────────────
$(UNITY_C):
	@echo "Downloading Unity $(UNITY_VERSION)..."
	$(MKDIR_P)
	$(DOWNLOAD)

# ── Run all test suites ───────────────────────────────────────────────────────
test: $(UNITY_C)
	@FAIL=0; \
	for t in test_*.cpp; do \
	  name=$$(basename $$t .cpp); \
	  $(CXX) $(CXXFLAGS) $(SRC_FILES) $(STUB_FILES) $(UNITY_C) $$t -o /tmp/$$name 2>&1; \
	  echo "=== $$name ==="; \
	  /tmp/$$name || FAIL=$$((FAIL + 1)); \
	done; \
	exit $$FAIL

clean:
	rm -rf $(UNITY_DIR)
	rm -f $(addprefix /tmp/,$(basename $(notdir $(wildcard test_*.cpp))))
