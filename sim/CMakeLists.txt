add_executable(sim
    main.cpp
    environment.cpp
    sensors.cpp
    simulation.cpp
    initialization.cpp
    data/file_writer.cpp
    data/telemetry_writer.cpp
)
set_property(TARGET sim PROPERTY CXX_STANDARD 17)

include(FetchContent)
include(ExternalProject)

if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()
FetchContent_Declare(
    apache_tsfile
    GIT_REPOSITORY https://github.com/apache/tsfile.git
    GIT_TAG v2.5.0
    GIT_SHALLOW TRUE
)
# TsFile's CMake assumes its source tree is the top-level project.
FetchContent_GetProperties(apache_tsfile)
if(NOT apache_tsfile_POPULATED)
    FetchContent_Populate(apache_tsfile)
endif()

set(APACHE_TSFILE_BINARY_DIR "${CMAKE_BINARY_DIR}/apache_tsfile-build")
set(APACHE_TSFILE_LIBRARY "${APACHE_TSFILE_BINARY_DIR}/lib/libtsfile.so")
ExternalProject_Add(apache_tsfile_build
    SOURCE_DIR "${apache_tsfile_SOURCE_DIR}/cpp"
    BINARY_DIR "${APACHE_TSFILE_BINARY_DIR}"
    CMAKE_ARGS
        "-DCMAKE_BUILD_TYPE=Release"
        "-DTSFILE_BUILD_SHARED=ON"
        "-DBUILD_TEST=OFF"
        "-DBUILD_TOOLS=OFF"
        "-DBUILD_BENCHMARK=OFF"
        "-DENABLE_ANTLR4=OFF"
        "-DENABLE_SNAPPY=OFF"
        "-DENABLE_LZ4=OFF"
        "-DENABLE_LZOKAY=OFF"
        "-DENABLE_ZLIB=OFF"
        "-DENABLE_ZSTD=OFF"
        "-DENABLE_LZMA2=OFF"
        "-DENABLE_SIMD=OFF"
    BUILD_COMMAND
        "${CMAKE_COMMAND}" --build <BINARY_DIR> --target tsfile --config Release --parallel 4
    BUILD_BYPRODUCTS "${APACHE_TSFILE_LIBRARY}"
    INSTALL_COMMAND ""
)
add_library(tsfile SHARED IMPORTED GLOBAL)
set_target_properties(tsfile PROPERTIES
    IMPORTED_LOCATION "${APACHE_TSFILE_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${apache_tsfile_SOURCE_DIR}/cpp/src"
)
add_dependencies(sim apache_tsfile_build)
set_property(TARGET sim APPEND PROPERTY BUILD_RPATH "${APACHE_TSFILE_BINARY_DIR}/lib")

configure_file(${PROJECT_SOURCE_DIR}/config.json config.json COPYONLY)

set(EIGEN_INCLUDE_DIR
    ${PROJECT_SOURCE_DIR}/shared/external/eigen-5.0.0/eigen-5.0.0
)

if(NOT EXISTS "${EIGEN_INCLUDE_DIR}/Eigen/Dense")
    message(FATAL_ERROR "Vendored Eigen headers were not found: ${EIGEN_INCLUDE_DIR}")
endif()

target_include_directories(sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/shared
    ${EIGEN_INCLUDE_DIR}
)
target_link_libraries(sim PRIVATE gnc_lib tsfile)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(aocs_orbit MODULE
    bindings/astro_bindings.cpp
)
target_include_directories(aocs_orbit PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/shared
    ${EIGEN_INCLUDE_DIR}
)
set_target_properties(aocs_orbit PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
)

if(BUILD_TESTING)
    add_executable(sim_math_tests
        tests/math_tests.cpp
        sensors.cpp
    )
    target_include_directories(sim_math_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/shared
        ${EIGEN_INCLUDE_DIR}
    )
    target_link_libraries(sim_math_tests PRIVATE gnc_lib)
    add_test(NAME sim_math_tests COMMAND sim_math_tests)
endif()

find_package(jsoncpp CONFIG QUIET)

if(TARGET JsonCpp::JsonCpp)
    target_link_libraries(sim PRIVATE JsonCpp::JsonCpp)
elseif(TARGET jsoncpp_lib)
    target_link_libraries(sim PRIVATE jsoncpp_lib)
elseif(TARGET jsoncpp_static)
    target_link_libraries(sim PRIVATE jsoncpp_static)
else()
    find_path(JSONCPP_INCLUDE_DIR NAMES json/value.h PATH_SUFFIXES jsoncpp)
    find_library(JSONCPP_LIBRARY NAMES jsoncpp)

    if(NOT JSONCPP_INCLUDE_DIR OR NOT JSONCPP_LIBRARY)
        message(FATAL_ERROR "JsonCpp was not found. Install its development headers and library.")
    endif()

    add_library(JsonCpp::JsonCpp UNKNOWN IMPORTED)
    set_target_properties(JsonCpp::JsonCpp PROPERTIES
        IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}"
    )
    target_link_libraries(sim PRIVATE JsonCpp::JsonCpp)
endif()