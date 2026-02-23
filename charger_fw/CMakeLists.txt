cmake_minimum_required(VERSION 3.16)

# Tell ESP-IDF where to find extra components
# Each subdirectory listed here is scanned for components
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}/src/drivers"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(charger_fw)
