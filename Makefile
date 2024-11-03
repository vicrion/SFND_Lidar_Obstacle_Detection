SHELL := /bin/bash
LINUX_RELEASE := $(shell lsb_release -c | cut -f2)

BASE.DIR=$(PWD)
BUILD.DIR=$(BASE.DIR)/build
VCPKG.ROOT=$(BASE.DIR)/vcpkg
VCPKG.BIN=$(VCPKG.ROOT)/vcpkg

vcpkg: .FORCE
	rm -rf $(VCPKG.ROOT) && git clone https://github.com/microsoft/vcpkg.git && cd $(BASE.DIR)/vcpkg && ./bootstrap-vcpkg.sh -disableMetrics

baseline: .FORCE
	$(VCPKG.BIN) x-update-baseline --add-initial-baseline

# dynamic build will allow for debugging capability
TRIPLET.NAME=x64-linux-dynamic
ifdef CIRCLECI # static build for CCI
	TRIPLET.NAME=x64-linux-release
endif
libraries: .FORCE
	export VCPKG_BINARY_SOURCES=$(CACHE) && export VCPKG_BUILD_TYPE=release && $(VCPKG.BIN) install --vcpkg-root=$(VCPKG.ROOT) --enforce-port-checks --host-triplet=$(TRIPLET.NAME) --triplet=$(TRIPLET.NAME)

build:
	rm -rf $(BUILD.DIR) && mkdir -p $(BUILD.DIR)
	cd $(BUILD.DIR) && cmake .. && make

.FORCE: