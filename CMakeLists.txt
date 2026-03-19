cmake_minimum_required(VERSION 3.20)

project(NURBS VERSION 0.1.0 LANGUAGES CXX)

option(NURBS_BUILD_EXAMPLES "Build example programs" ON)
option(NURBS_BUILD_TESTS "Build test programs" ON)

add_library(nurbs
    src/core/KnotVector.cpp
    src/basis/Basis.cpp
    src/geometry/NurbsCurve.cpp
)

target_include_directories(nurbs
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(nurbs PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(nurbs PRIVATE /W4 /permissive-)
else()
    target_compile_options(nurbs PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(NURBS_BUILD_EXAMPLES)
    add_executable(evaluate_curve examples/evaluate_curve.cpp)
    target_link_libraries(evaluate_curve PRIVATE nurbs)
endif()

if(NURBS_BUILD_TESTS)
    enable_testing()

    add_executable(test_nurbs tests/test_nurbs.cpp)
    target_link_libraries(test_nurbs PRIVATE nurbs)

    add_test(NAME test_nurbs COMMAND test_nurbs)
endif()
