cmake_minimum_required(VERSION 3.16.3)
project(simHaply)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

include(ExternalProject)
set(HWAPI_VER 0.1.12)
if(WIN32)
    set(HWAPI_URL https://cdn.haply.co/r/33857671/HardwareAPI.${HWAPI_VER}.win32.zip)
elseif(APPLE)
    set(HWAPI_URL https://cdn.haply.co/r/33857671/HardwareAPI.${HWAPI_VER}.macos.tar.gz)
elseif(UNIX)
    execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
    if(ARCH STREQUAL aarch64)
        set(HWAPI_URL https://cdn.haply.co/r/33857671/HardwareAPI.${HWAPI_VER}.linux.gcc10.aarch64.tar.gz)
    else()
        set(HWAPI_URL https://cdn.haply.co/r/33857671/HardwareAPI.${HWAPI_VER}.linux.tar.gz)
    endif()
endif()
set(HWAPI_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/hwapi-${HWAPI_VER}-prefix)
set(HWAPI_SRC_DIR ${HWAPI_PREFIX}/src/hwapi-${HWAPI_VER})
set(HWAPI_BUILD_DIR ${HWAPI_PREFIX}/src/hwapi-${HWAPI_VER}-build)
ExternalProject_Add(hwapi-${HWAPI_VER}
    URL ${HWAPI_URL}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${HWAPI_SRC_DIR}/include)
link_directories(${HWAPI_SRC_DIR})

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/sourceCode/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lua/simHaply.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/sourceCode/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h ESCAPE_QUOTES)

coppeliasim_add_plugin(simHaply
    SOURCES
    sourceCode/plugin.cpp
    sourceCode/haply_inverse3.cpp
    sourceCode/haply_handle.cpp
    sourceCode/control_loop.cpp
)
add_dependencies(simHaply hwapi-${HWAPI_VER})
target_link_libraries(simHaply PRIVATE Haply.HardwareAPI)
target_link_libraries(simHaply PRIVATE Eigen3::Eigen)

coppeliasim_add_lua(lua/models/Haply_device_customization-1.lua SUBDIR models)

add_executable(haply_test sourceCode/haply_test.cpp)
add_dependencies(haply_test hwapi-${HWAPI_VER})
target_link_libraries(haply_test PRIVATE Haply.HardwareAPI)
