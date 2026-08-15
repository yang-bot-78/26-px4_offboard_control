cmake_minimum_required(VERSION 3.15...4.3)

project(ikd_tree VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_EXAMPLES "Build the synthetic kNN demo" OFF)
option(BUILD_BENCHMARKS "Build kNN benchmarks" OFF)

# The library only uses pcl::point_types; the `common` component is enough.
find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(Threads REQUIRED)

# Workaround: conda-forge's boost_headers-config.cmake hardcodes
# `_BOOST_CMAKEDIR` to the build-time runner path (e.g.
# /Users/runner/miniforge3/conda-bld/.../temp_prefix/lib/cmake), so
# Boost::headers' INTERFACE_INCLUDE_DIRECTORIES points at a path that
# does not exist on the user's machine. Until that upstream packaging
# bug is fixed, inject the bare conda env include dir manually.
if(NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
    add_compile_options("-I$ENV{CONDA_PREFIX}/include")
endif()

add_library(ikd_tree STATIC src/ikd_tree.cpp)
add_library(ikd_tree::ikd_tree ALIAS ikd_tree)

target_compile_features(ikd_tree PUBLIC cxx_std_17)
set_target_properties(ikd_tree PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    POSITION_INDEPENDENT_CODE ON
)
target_compile_options(ikd_tree PRIVATE -O3)

target_include_directories(ikd_tree
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
foreach(_pcl_inc ${PCL_INCLUDE_DIRS})
    target_include_directories(ikd_tree SYSTEM PUBLIC $<BUILD_INTERFACE:${_pcl_inc}>)
endforeach()
target_link_libraries(ikd_tree PUBLIC ${PCL_LIBRARIES} Threads::Threads)

if(BUILD_EXAMPLES)
    add_executable(ikd_tree_demo examples/ikd_tree_demo.cpp)
    target_link_libraries(ikd_tree_demo PRIVATE ikd_tree::ikd_tree)
    install(TARGETS ikd_tree_demo RUNTIME DESTINATION bin)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    add_executable(ikd_tree_tests tests/test_ikd_tree.cpp)
    target_link_libraries(ikd_tree_tests PRIVATE ikd_tree::ikd_tree GTest::gtest_main)
    add_test(NAME ikd_tree_tests COMMAND ikd_tree_tests)
endif()

if(BUILD_BENCHMARKS)
    find_package(benchmark REQUIRED)
    add_executable(ikd_tree_bench bench/bench_ikd_tree.cpp)
    target_link_libraries(ikd_tree_bench PRIVATE ikd_tree::ikd_tree benchmark::benchmark_main)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ikd_tree
    EXPORT ikd_treeTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ikd_treeTargets
    NAMESPACE ikd_tree::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ikd_tree
    FILE ikd_treeTargets.cmake
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ikd_treeConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ikd_treeConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ikd_treeConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ikd_tree
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ikd_treeConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ikd_treeConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ikd_tree
)
