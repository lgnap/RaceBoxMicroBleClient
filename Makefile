# RaceBoxMicroBleClient — Root Makefile
#
# Usage:
#   make          Run native unit tests (87 tests, no hardware required)
#   make test     Same as above
#   make clean    Remove compiled test binaries
#   make help     Show this help

.PHONY: all test clean help

all: test

test:
	$(MAKE) -C test

clean:
	$(MAKE) -C test clean

help:
	@echo "RaceBoxMicroBleClient"
	@echo ""
	@echo "  make / make test   Run native unit tests (87 tests, g++ required)"
	@echo "  make clean         Remove compiled test binaries"
	@echo ""
	@echo "Physical device tests: see TESTING.md"
