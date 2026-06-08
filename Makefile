.PHONY: debug reldebug release clean help d rd r h
BUILD_BACKEND ?= Ninja
BUILD_BACKEND_CMD = $(shell echo $(BUILD_BACKEND) | tr '[:upper:]' '[:lower:]')
BUILD_PATH ?= build

# Determine CMake generator based on availability of BUILD_BACKEND
ifneq ($(shell command -v $(BUILD_BACKEND_CMD) >/dev/null 2>&1 && echo yes),)
CMAKE_GENERATOR = -G $(BUILD_BACKEND)
else
CMAKE_GENERATOR =
endif

debug: 
	@echo "Building in debug mode..."
	cmake $(CMAKE_GENERATOR) -S . -B $(BUILD_PATH) -DCMAKE_COLOR_DIAGNOSTICS=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_CCACHE=YES
	cmake --build $(BUILD_PATH) --target clean
	cmake --build $(BUILD_PATH) --target FMACM -j

reldebug: 
	@echo "Building in release with debug info mode..."
	cmake $(CMAKE_GENERATOR) -S . -B $(BUILD_PATH) -DCMAKE_COLOR_DIAGNOSTICS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_CCACHE=YES
	cmake --build $(BUILD_PATH) --target clean
	cmake --build $(BUILD_PATH) --target FMACM -j

release: 
	@echo "Building in release mode..."
	cmake $(CMAKE_GENERATOR) -S . -B $(BUILD_PATH) -DCMAKE_COLOR_DIAGNOSTICS=ON -DCMAKE_BUILD_TYPE=Release -DUSE_CCACHE=YES
	cmake --build $(BUILD_PATH) --target clean
	cmake --build $(BUILD_PATH) --target FMACM -j

nuke:
	@echo "Removing build directory..."
	rm -rf $(BUILD_PATH)

tests:
	@echo "Running unit tests..."
	cmake $(CMAKE_GENERATOR) -S . -B $(BUILD_PATH) -DCMAKE_COLOR_DIAGNOSTICS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_CCACHE=YES
	cmake --build $(BUILD_PATH) --target run_tests -j --clean-first

help:
	@echo "Type less; dev more:"
	@echo "  debug (d)      : Build in debug mode"
	@echo "  reldebug (rd)  : Build in release with debug info mode"
	@echo "  release (r)    : Build in release mode"
	@echo "  nuke (n)       : Delete the build directory"
	@echo "  tests (ut)     : Run unit tests"
	@echo "  help (h)       : Show this help message"

d: debug
rd: reldebug
r: release
n: nuke
ut: tests
h: help
