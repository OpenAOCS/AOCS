cmake_minimum_required(VERSION 3.20)
project(AOCS VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-O2 -Wall -Wextra -Wpedantic")

option(AOCS_ENABLE_CLANG_TIDY "Run clang-tidy while compiling project targets" OFF)
if(AOCS_ENABLE_CLANG_TIDY)
    find_program(AOCS_CLANG_TIDY_EXECUTABLE NAMES clang-tidy)
    if(NOT AOCS_CLANG_TIDY_EXECUTABLE)
        message(FATAL_ERROR "AOCS_ENABLE_CLANG_TIDY is ON, but clang-tidy was not found.")
    endif()
endif()

function(aocs_enable_clang_tidy target config_file)
    if(AOCS_ENABLE_CLANG_TIDY)
        set_property(TARGET ${target} PROPERTY CXX_CLANG_TIDY
            "${AOCS_CLANG_TIDY_EXECUTABLE};--config-file=${config_file}"
        )
    endif()
endfunction()

include(CTest)

add_subdirectory(gnc_lib)
add_subdirectory(sim)