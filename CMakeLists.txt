cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/src")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ESP-IDF builds with -Werror=all by default; keep warnings as warnings
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
project(NeuralNetRobot)
