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

add_library(voxel4d_core STATIC
    src/doppler_simulator.cpp
    src/octree.cpp
    src/raytracer.cpp
    src/synthetic_data_generator.cpp
    src/temporal_voxel_map.cpp
    src/voxelizer.cpp
)
add_library(Voxel4D::core ALIAS voxel4d_core)

target_compile_features(voxel4d_core PUBLIC cxx_std_17)
target_include_directories(voxel4d_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(voxel4d_core PUBLIC glm::glm)

function(voxel4d_apply_build_settings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /permissive-)
        if(VOXEL4D_WARNINGS_AS_ERRORS)
            target_compile_options(${target_name} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
        if(VOXEL4D_WARNINGS_AS_ERRORS)
            target_compile_options(${target_name} PRIVATE -Werror)
        endif()
    endif()

    if(VOXEL4D_ENABLE_SANITIZERS AND NOT MSVC)
        target_compile_options(${target_name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        get_target_property(target_type ${target_name} TYPE)
        if(NOT target_type STREQUAL "STATIC_LIBRARY")
            target_link_options(${target_name} PRIVATE -fsanitize=address,undefined)
        endif()
    endif()
endfunction()

voxel4d_apply_build_settings(voxel4d_core)

add_executable(voxel4d_poc src/main.cpp)
add_executable(Voxel4D::poc ALIAS voxel4d_poc)
target_link_libraries(voxel4d_poc PRIVATE Voxel4D::core)
voxel4d_apply_build_settings(voxel4d_poc)

include(CTest)
if(BUILD_TESTING)
    add_executable(voxel4d_temporal_voxel_map_test tests/temporal_voxel_map_test.cpp)
    target_link_libraries(voxel4d_temporal_voxel_map_test PRIVATE Voxel4D::core)
    voxel4d_apply_build_settings(voxel4d_temporal_voxel_map_test)

    add_test(NAME voxel4d_temporal_voxel_map_test COMMAND voxel4d_temporal_voxel_map_test)
    add_test(NAME voxel4d_poc_smoke_test COMMAND voxel4d_poc)
    set_tests_properties(voxel4d_poc_smoke_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()
