#!/usr/bin/env bash
# Fetches Bluepad32/BTstack into ~/.espressif/bluepad32.
# components/procon depends on Bluepad32/BTstack, which are not vendored in
# this repo (see CMakeLists.txt / components/procon/CMakeLists.txt).
# Same steps as .github/workflows/esp32-build.yml ("Install Bluepad32").
# Keep this in sync with that workflow.
set -eu

BP32_DIR="$HOME/.espressif/bluepad32"
# Re-clone if missing, or if a previous attempt left an incomplete checkout
# in the cached ~/.espressif volume (e.g. interrupted by a network error
# partway through, or a submodule fetch failure) — checking ".git" alone
# isn't enough to prove the clone actually finished.
if [ ! -d "$BP32_DIR/.git" ] || [ ! -d "$BP32_DIR/src/components" ]; then
	rm -rf "$BP32_DIR"
	git clone --recurse-submodules https://github.com/ricardoquesada/bluepad32.git "$BP32_DIR"
fi

cd "$BP32_DIR/external/btstack/port/esp32"
# Installs BTstack as a component inside Bluepad32's own source tree
# (bluepad32/src/components), which is what EXTRA_COMPONENT_DIRS in
# ESP32/controller/CMakeLists.txt expects.
IDF_PATH=../../../../src python3 integrate_btstack.py

echo "Bluepad32 ready at $BP32_DIR"
