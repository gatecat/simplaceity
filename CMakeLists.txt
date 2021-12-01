cmake_minimum_required(VERSION 3.8)
project(simplaceity CXX C)

set(CMAKE_CXX_STANDARD 20)

set(WARN_FLAGS "-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wno-array-bounds")
set(CMAKE_CXX_FLAGS_DEBUG "${WARN_FLAGS} -fPIC -ggdb -pipe")
set(CMAKE_CXX_FLAGS_RELEASE "${WARN_FLAGS} -fPIC -O3 -g -pipe -fopenmp")


add_subdirectory(thirdparty/pybind11)
include_directories(core)
aux_source_directory(core CORE_FILES)
pybind11_add_module(simplaceity ${CORE_FILES})
