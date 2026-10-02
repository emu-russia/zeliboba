cmake_minimum_required(VERSION 3.20)

project(zeliboba
        VERSION 0.1.0
        DESCRIPTION "Low-level PlayStation Vita emulator (CMeP/ARM/Ernie), firmware 1.04 bring-up"
        LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global build settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# The executables carry the artwork from artwork/make_artwork.py as a Windows
# icon resource, so the resource compiler has to be available.
if(MSVC)
    enable_language(RC)
endif()

option(ZLB_BUILD_TOOLS   "Build command line tools (emmc_rebuild, zdis)" ON)
option(ZLB_BUILD_TESTS   "Build self tests"                              ON)
option(ZLB_ENABLE_SDL3   "Build the SDL3 frontend (falls back to headless when unavailable)" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY        ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG  ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY        ${CMAKE_BINARY_DIR}/bin)

if(MSVC)
    add_compile_options(/utf-8 /permissive- /Zc:__cplusplus /W3 /MP /EHsc)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/std:c++20>)
    # keep the interpreter honest: no fast-math style reassociation of flag logic
    add_compile_options(/fp:precise)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# ---------------------------------------------------------------------------
# SDL3
# ---------------------------------------------------------------------------
set(ZLB_SDL3_AVAILABLE OFF)

if(ZLB_ENABLE_SDL3)
    # 1) an installed SDL3 found through the CMake package registry
    find_package(SDL3 QUIET CONFIG)
    if(SDL3_FOUND)
        set(ZLB_SDL3_AVAILABLE ON)
        message(STATUS "SDL3: found via find_package")
    elseif(WIN32)
        # 2) a vendored developer package under third_party/SDL3-devel-VC
        file(GLOB ZLB_SDL3_VC_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SDL3-devel-VC"
                                   "${CMAKE_CURRENT_SOURCE_DIR}/third_party/SDL3-devel-*-VC")
        foreach(dir ${ZLB_SDL3_VC_DIRS})
            if(EXISTS "${dir}/include/SDL3/SDL.h")
                # Pick the architecture matching the build: x64 on MSVC x64.
                if(CMAKE_SIZEOF_VOID_P EQUAL 8)
                    set(_sdl3arch "x64")
                else()
                    set(_sdl3arch "x86")
                endif()
                set(_sdl3lib "${dir}/lib/${_sdl3arch}/SDL3.lib")
                if(EXISTS "${_sdl3lib}")
                    add_library(SDL3::SDL3 SHARED IMPORTED GLOBAL)
                    set_target_properties(SDL3::SDL3 PROPERTIES
                        IMPORTED_LOCATION "${dir}/lib/${_sdl3arch}/SDL3.dll"
                        IMPORTED_IMPLIB   "${_sdl3lib}"
                        INTERFACE_INCLUDE_DIRECTORIES "${dir}/include")
                    set(ZLB_SDL3_AVAILABLE ON)
                    message(STATUS "SDL3: vendored developer package at ${dir} (${_sdl3arch})")
                endif()
            endif()
        endforeach()
    endif()

    if(NOT ZLB_SDL3_AVAILABLE)
        message(STATUS "SDL3: not available - building the headless frontend only")
    endif()
endif()

# ---------------------------------------------------------------------------
# zeliboba_core : everything that does not depend on a windowing system
# ---------------------------------------------------------------------------
file(GLOB_RECURSE ZLB_CORE_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/bus/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/cpu/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/hw/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/loader/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/machine/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/debug/*.cpp")

add_library(zeliboba_core STATIC ${ZLB_CORE_SOURCES})
target_include_directories(zeliboba_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_definitions(zeliboba_core PUBLIC
    ZLB_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    ZLB_WORKSPACE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")

# ---------------------------------------------------------------------------
# zeliboba (headless CLI emulator / debugger)
# ---------------------------------------------------------------------------
add_executable(zeliboba "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
target_link_libraries(zeliboba PRIVATE zeliboba_core)
if(MSVC)
    # artwork/zeliboba.ico + version info (src/zeliboba.rc)
    target_sources(zeliboba PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/zeliboba.rc")
endif()

# ---------------------------------------------------------------------------
# zeliboba_ui (SDL3 frontend), only when SDL3 is present
# ---------------------------------------------------------------------------
if(ZLB_SDL3_AVAILABLE)
    file(GLOB_RECURSE ZLB_UI_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/*.cpp")
    if(ZLB_UI_SOURCES)
        add_executable(zeliboba_ui ${ZLB_UI_SOURCES})
        target_link_libraries(zeliboba_ui PRIVATE zeliboba_core SDL3::SDL3)
        target_compile_definitions(zeliboba_ui PRIVATE ZLB_HAVE_SDL3=1)
        if(MSVC)
            # artwork/zeliboba.ico + version info (src/ui/zeliboba_ui.rc)
            target_sources(zeliboba_ui PRIVATE
                           "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/zeliboba_ui.rc")
        endif()
        if(WIN32)
            # Windows needs the runtime DLL beside the executable. Installed
            # macOS/Linux SDL3 packages already carry their runtime paths.
            add_custom_command(TARGET zeliboba_ui POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_FILE:SDL3::SDL3> $<TARGET_FILE_DIR:zeliboba_ui>
                COMMENT "Copying SDL3.dll next to zeliboba_ui")
        endif()
    else()
        message(STATUS "SDL3 present but src/ui is empty - frontend skipped")
    endif()
endif()

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
if(ZLB_BUILD_TOOLS)
    file(GLOB ZLB_TOOL_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/*.cpp")
    foreach(tool_src ${ZLB_TOOL_SOURCES})
        get_filename_component(tool_name "${tool_src}" NAME_WE)
        add_executable(${tool_name} "${tool_src}")
        target_link_libraries(${tool_name} PRIVATE zeliboba_core)
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(ZLB_BUILD_TESTS)
    enable_testing()
    file(GLOB ZLB_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    if(ZLB_TEST_SOURCES)
        add_executable(zlb_tests ${ZLB_TEST_SOURCES})
        target_link_libraries(zlb_tests PRIVATE zeliboba_core)
        target_include_directories(zlb_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tests")
        add_test(NAME zlb_tests COMMAND zlb_tests)
    endif()
endif()

message(STATUS "zeliboba: SDL3 frontend ${ZLB_SDL3_AVAILABLE}")
