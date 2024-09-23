cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(minisumo_esp32_official)
set_source_files_properties(components/vl530x/platform/inc/vl53l0x_platform.c PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)