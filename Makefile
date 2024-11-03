SHELL := /bin/bash
LINUX_RELEASE := $(shell lsb_release -c | cut -f2)

BASE.DIR=$(PWD)
DOWNLOADS.DIR=$(BASE.DIR)/downloads
VCPKG.ROOT=$(BASE.DIR)/vcpkg
VCPKG.BIN=$(VCPKG.ROOT)/vcpkg