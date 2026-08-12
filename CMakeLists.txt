cmake_minimum_required(VERSION 3.16)

project(Voxel4D
    VERSION 0.1.0
    DESCRIPTION "A C++17 proof of concept for sparse voxel video reconstruction"
    LANGUAGES CXX)

option(VOXEL4D_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(VOXEL4D_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

find_package(glm CONFIG QUIET)
if(NOT TARGET glm::glm)
    find_path(GLM_INCLUDE_DIR NAMES glm/glm.hpp)
    if(NOT GLM_INCLUDE_DIR)
        message(FATAL_ERROR "GLM was not found. Install GLM or provide a CMake package named glm::glm.")
    endif()
    add_library(glm::glm INTERFACE IMPORTED)
    target_include_directories(glm::glm INTERFACE ${GLM_INCLUDE_DIR})
endif()

add_executable(voxel4d_poc
    src/main.cpp
    src/octree.cpp
    src/voxelizer.cpp
    src/raytracer.cpp
    src/doppler_simulator.cpp
    src/synthetic_data_generator.cpp
)

add_executable(Voxel4D::poc ALIAS voxel4d_poc)

target_compile_features(voxel4d_poc PRIVATE cxx_std_17)

target_include_directories(voxel4d_poc
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(voxel4d_poc PRIVATE glm::glm)

if(MSVC)
    target_compile_options(voxel4d_poc PRIVATE /W4 /permissive-)
    if(VOXEL4D_WARNINGS_AS_ERRORS)
        target_compile_options(voxel4d_poc PRIVATE /WX)
    endif()
else()
    target_compile_options(voxel4d_poc PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    if(VOXEL4D_WARNINGS_AS_ERRORS)
        target_compile_options(voxel4d_poc PRIVATE -Werror)
    endif()
endif()

if(VOXEL4D_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(voxel4d_poc PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(voxel4d_poc PRIVATE -fsanitize=address,undefined)
endif()

include(CTest)
if(BUILD_TESTING)
    add_test(NAME voxel4d_poc_smoke_test COMMAND voxel4d_poc)
    set_tests_properties(voxel4d_poc_smoke_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()
