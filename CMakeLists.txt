cmake_minimum_required(VERSION 2.6)
project(HRBF-skin-plugin)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Set up include and lib/external paths
set(EXTERNAL "external")
include_directories("${EXTERNAL}/include")

add_subdirectory(src)