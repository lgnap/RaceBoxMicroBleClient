# RaceBoxMicroBleClient — Root Makefile
#
# Native tests (no hardware):
#   make                   Run all 87 unit tests
#   make test              Same
#   make clean             Remove compiled test binaries
#
# Build & flash .ino examples (arduino-cli required):
#   make build SKETCH=LibTest           Compile examples/LibTest/LibTest.ino
#   make flash SKETCH=LibTest PORT=/dev/ttyUSB0   Compile + upload
#   make monitor PORT=/dev/ttyUSB0     Open serial monitor (115200 baud)
#
# arduino-cli install: https://arduino.github.io/arduino-cli/latest/installation/
# Board setup (once):  make setup-board

# ── Config ────────────────────────────────────────────────────────────────────
ARDUINO_CLI ?= arduino-cli
FQBN        ?= esp32:esp32:ttgo-lora32
PORT        ?= /dev/ttyUSB0
BAUD        ?= 115200
SKETCH      ?= LibTest

SKETCH_DIR  := examples/$(SKETCH)
SKETCH_FILE := $(SKETCH_DIR)/$(SKETCH).ino
BUILD_DIR   := /tmp/arduino-build-$(SKETCH)

.PHONY: all test clean build flash monitor setup-board help

# ── Default: run unit tests ───────────────────────────────────────────────────
all: test

test:
	$(MAKE) -C test

clean:
	$(MAKE) -C test clean
	rm -rf /tmp/arduino-build-*

# ── Board setup (run once after installing arduino-cli) ───────────────────────
setup-board:
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install esp32:esp32
	$(ARDUINO_CLI) lib install "ESP32 BLE Arduino"

# ── Compile a sketch ─────────────────────────────────────────────────────────
build: _check-sketch
	$(ARDUINO_CLI) compile \
	  --fqbn $(FQBN) \
	  --libraries . \
	  --build-path $(BUILD_DIR) \
	  $(SKETCH_FILE)

# ── Compile + upload ──────────────────────────────────────────────────────────
flash: _check-sketch
	$(ARDUINO_CLI) compile \
	  --fqbn $(FQBN) \
	  --libraries . \
	  --build-path $(BUILD_DIR) \
	  --upload \
	  --port $(PORT) \
	  $(SKETCH_FILE)

# ── Serial monitor ────────────────────────────────────────────────────────────
monitor:
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=$(BAUD)

# ── Internal guard ────────────────────────────────────────────────────────────
_check-sketch:
	@test -f "$(SKETCH_FILE)" || { \
	  echo "ERROR: $(SKETCH_FILE) not found."; \
	  echo "Available sketches: $(notdir $(wildcard examples/*/))"; \
	  exit 1; \
	}

# ── Help ─────────────────────────────────────────────────────────────────────
help:
	@echo "RaceBoxMicroBleClient"
	@echo ""
	@echo "Native tests (no hardware):"
	@echo "  make                          Run all 87 unit tests"
	@echo "  make clean                    Remove compiled binaries"
	@echo ""
	@echo "Build & flash (arduino-cli required):"
	@echo "  make setup-board              Install ESP32 core + deps (once)"
	@echo "  make build SKETCH=LibTest     Compile a sketch"
	@echo "  make flash  SKETCH=LibTest PORT=/dev/ttyUSB0   Compile + upload"
	@echo "  make monitor PORT=/dev/ttyUSB0                 Serial monitor"
	@echo ""
	@echo "Available sketches: $(notdir $(wildcard examples/*/*))"
	@echo "Default FQBN: $(FQBN)  (override with FQBN=...)"
