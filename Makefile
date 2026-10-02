# Shortcuts for the PC-side checks. Run `make test` to run everything.
#
#   make test-host    build and run the C++ unit tests (no ESP32 needed)
#   make test-python  run the Python tests
#   make firmware     compile the ESP32 firmware
#   make demo         run every analysis script on synthetic demo data
#   make reference    rewrite tests/reference_io/ from the C++ code (after changing it)

CXX ?= g++
# -ffp-contract=off: never fuse a*b+c into one instruction, so the C++ and
# Python estimators produce the same numbers (tests/test_estimators.py).
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Werror -ffp-contract=off -Ifirmware/include
PYTHON ?= python3

.PHONY: test test-host test-python firmware reference demo

test: test-host test-python

build/test_host: tests/test_host.cpp firmware/include/*.h
	mkdir -p build
	$(CXX) $(CXXFLAGS) tests/test_host.cpp -o build/test_host

test-host: build/test_host
	./build/test_host

test-python:
	$(PYTHON) -m pytest -q tests

firmware:
	pio run

reference: build/test_host
	./build/test_host --reference tests/reference_io

demo:
	$(PYTHON) analysis/demo/run_demo.py
