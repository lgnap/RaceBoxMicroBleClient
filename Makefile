# RaceBoxMicroBleClient — Root Makefile
#
# Native tests (no hardware):
#   make                   Run all 87 unit tests
#   make test              Same
#   make clean             Remove compiled test binaries + Unity cache
#
# Build & flash .ino examples (arduino-cli required):
#   make build SKETCH=LibTest                      Compile
#   make flash SKETCH=LibTest PORT=COM3            Compile + upload
#   make monitor PORT=COM3                         Serial monitor (115200 baud)
#   make setup-board                               Install ESP32 core (once)
#
# arduino-cli: https://arduino.github.io/arduino-cli/latest/installation/

# On Windows, force Git Bash (sh.exe) so bash-style commands work.
ifeq ($(OS),Windows_NT)
  SHELL := sh
endif

# ── Config ────────────────────────────────────────────────────────────────────
ARDUINO_CLI ?= arduino-cli
FQBN        ?= esp32:esp32:ttgo-lora32
PORT        ?= /dev/ttyUSB0
BAUD        ?= 115200
SKETCH      ?= LibTest

SKETCH_DIR  := examples/$(SKETCH)
SKETCH_FILE := $(SKETCH_DIR)/$(SKETCH).ino
BUILD_DIR   := /tmp/arduino-build-$(SKETCH)

.PHONY: all test clean build flash monitor setup-board _check-sketch help

# ── Default: run unit tests ───────────────────────────────────────────────────
all: test

test:
	$(MAKE) -C test

clean:
	$(MAKE) -C test clean

# ── Board setup (run once after installing arduino-cli) ───────────────────────
setup-board:
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install esp32:esp32
	$(ARDUINO_CLI) lib install "ESP32 BLE Arduino"

# ── Compile a sketch ──────────────────────────────────────────────────────────
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

# ── Internal guard (pure make — no shell needed) ──────────────────────────────
_check-sketch:
	$(if $(wildcard $(SKETCH_FILE)),,$(error Sketch not found: $(SKETCH_FILE). Available: $(notdir $(wildcard examples/*/*))))

# ── Help ──────────────────────────────────────────────────────────────────────
help:
	@echo "RaceBoxMicroBleClient"
	@echo ""
	@echo "Native tests (no hardware):"
	@echo "  make                          Run all 87 unit tests"
	@echo "  make clean                    Remove compiled binaries"
	@echo ""
	@echo "Build and flash (arduino-cli required):"
	@echo "  make setup-board              Install ESP32 core + deps (once)"
	@echo "  make build SKETCH=LibTest     Compile a sketch"
	@echo "  make flash  SKETCH=LibTest PORT=COM3   Compile + upload"
	@echo "  make monitor PORT=COM3                 Serial monitor"
	@echo ""
	@echo "Default FQBN: $(FQBN)  (override with FQBN=...)"
