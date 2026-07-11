# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# ToDo: set your bp32root
set(BLUEPAD32_ROOT "C:/Espressif/.espressif/bluepad32")

set(BTSTACK_ROOT ${BLUEPAD32_ROOT}/external/btstack)
set(EXTRA_COMPONENT_DIRS ${BLUEPAD32_ROOT}/src/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(controller)