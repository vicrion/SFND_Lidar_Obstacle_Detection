SHELL := /bin/bash
LINUX_RELEASE := $(shell lsb_release -c | cut -f2)

BASE.DIR=$(PWD)
VCPKG.ROOT=$(BASE.DIR)/vcpkg
VCPKG.BIN=$(VCPKG.ROOT)/vcpkg

vcpkg: .FORCE
	rm -rf $(VCPKG.ROOT) && git clone https://github.com/microsoft/vcpkg.git && cd $(BASE.DIR)/vcpkg && ./bootstrap-vcpkg.sh -disableMetrics

baseline: .FORCE
	$(VCPKG.BIN) x-update-baseline --add-initial-baseline

# dynamic build will allow for debugging capability
TRIPLET.NAME=x64-linux
libraries: .FORCE
	$(VCPKG.BIN) install --vcpkg-root=$(VCPKG.ROOT) --enforce-port-checks --host-triplet=$(TRIPLET.NAME) --triplet=$(TRIPLET.NAME)

BUILD.DIR=$(BASE.DIR)/build_cmd
build: .FORCE
	rm -rf $(BUILD.DIR) && mkdir -p $(BUILD.DIR)
	cmake $(BASE.DIR) -B$(BUILD.DIR) -DCMAKE_TOOLCHAIN_FILE=$(VCPKG.ROOT)/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=$(TRIPLET.NAME) -DVCPKG_HOST_TRIPLET=$(TRIPLET.NAME) -DVCPKG_BUILD_TYPE=release -DVCPKG_INSTALLED_DIR=$(BASE.DIR)/vcpkg_installed -DCMAKE_INSTALL_PREFIX=$(INSTALL.DIR) -DBUILD_SHARED_LIBS=0 -DSKIP_TESTS=0 && \
	cd $(BUILD.DIR) && cmake --build .

APP.BIN=$(BUILD.DIR)/environment
run: .FORCE
	export LD_LIBRARY_PATH=$(BASE.DIR)/vcpkg_installed/$(TRIPLET.NAME)/lib:$(BASE.DIR)/vcpkg_installed/$(TRIPLET.NAME)/debug/lib:$(INSTALL.DIR)/lib && \
	$(APP.BIN)

.FORCE: