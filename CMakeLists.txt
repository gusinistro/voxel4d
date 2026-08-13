cmake_minimum_required(VERSION 3.16)

project(Voxel4D
    VERSION 0.1.0
    DESCRIPTION "A C++17 proof of concept for sparse voxel video reconstruction"
    LANGUAGES CXX)

option(VOXEL4D_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(VOXEL4D_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(VOXEL4D_BUILD_BENCHMARKS "Build reproducible CPU benchmark executables" ON)

find_package(glm CONFIG QUIET)
if(NOT TARGET glm::glm)
    find_path(GLM_INCLUDE_DIR NAMES glm/glm.hpp)
    if(NOT GLM_INCLUDE_DIR)
        message(FATAL_ERROR "GLM was not found. Install GLM or provide a CMake package named glm::glm.")
    endif()
    add_library(glm::glm INTERFACE IMPORTED)
    target_include_directories(glm::glm INTERFACE ${GLM_INCLUDE_DIR})
endif()

find_package(Threads REQUIRED)

add_library(voxel4d_core STATIC
    src/acoustic_raytracer.cpp
    src/doppler_simulator.cpp
    src/execution_runtime.cpp
    src/free_view_renderer.cpp
    src/multi_sensor_fuser.cpp
    src/multiview_calibration.cpp
    src/multiview_geometry.cpp
    src/object_tracker.cpp
    src/octree.cpp
    src/recorded_observation_csv.cpp
    src/raytracer.cpp
    src/semantic_inference.cpp
    src/sensor_observation.cpp
    src/sensor_pose.cpp
    src/sensor_pose_timeline.cpp
    src/spherical_harmonics.cpp
    src/synthetic_data_generator.cpp
    src/synthetic_sensor_adapter.cpp
    src/temporal_voxel_map.cpp
    src/visual_odometry.cpp
    src/voxelizer.cpp
)
add_library(Voxel4D::core ALIAS voxel4d_core)

target_compile_features(voxel4d_core PUBLIC cxx_std_17)
target_include_directories(voxel4d_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(voxel4d_core PUBLIC glm::glm Threads::Threads)

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

if(VOXEL4D_BUILD_BENCHMARKS)
    add_executable(voxel4d_render_benchmark benchmarks/render_benchmark.cpp)
    target_link_libraries(voxel4d_render_benchmark PRIVATE Voxel4D::core)
    voxel4d_apply_build_settings(voxel4d_render_benchmark)
endif()

include(CTest)
if(BUILD_TESTING)
    function(voxel4d_add_unit_test target_name source_file)
        add_executable(${target_name} ${source_file})
        target_link_libraries(${target_name} PRIVATE Voxel4D::core)
        target_include_directories(${target_name} PRIVATE ${PROJECT_SOURCE_DIR}/tests)
        voxel4d_apply_build_settings(${target_name})
        add_test(NAME ${target_name} COMMAND ${target_name})
    endfunction()

    voxel4d_add_unit_test(voxel4d_temporal_voxel_map_test tests/temporal_voxel_map_test.cpp)
    voxel4d_add_unit_test(voxel4d_spatial_core_test tests/spatial_core_test.cpp)
    voxel4d_add_unit_test(voxel4d_doppler_simulator_test tests/doppler_simulator_test.cpp)
    voxel4d_add_unit_test(voxel4d_sensor_pose_test tests/sensor_pose_test.cpp)
    voxel4d_add_unit_test(voxel4d_visual_odometry_test tests/visual_odometry_test.cpp)
    voxel4d_add_unit_test(voxel4d_sensor_observation_test tests/sensor_observation_test.cpp)
    voxel4d_add_unit_test(voxel4d_multi_sensor_fuser_test tests/multi_sensor_fuser_test.cpp)
    voxel4d_add_unit_test(voxel4d_acoustic_and_sh_test tests/acoustic_and_sh_test.cpp)
    voxel4d_add_unit_test(voxel4d_execution_runtime_test tests/execution_runtime_test.cpp)
    voxel4d_add_unit_test(voxel4d_multiview_calibration_test tests/multiview_calibration_test.cpp)
    voxel4d_add_unit_test(voxel4d_multiview_geometry_test tests/multiview_geometry_test.cpp)
    voxel4d_add_unit_test(voxel4d_object_tracker_test tests/object_tracker_test.cpp)
    voxel4d_add_unit_test(voxel4d_temporal_voxel_fusion_test tests/temporal_voxel_fusion_test.cpp)
    voxel4d_add_unit_test(voxel4d_free_view_renderer_test tests/free_view_renderer_test.cpp)
    voxel4d_add_unit_test(voxel4d_recorded_observation_csv_test tests/recorded_observation_csv_test.cpp)
    voxel4d_add_unit_test(voxel4d_semantic_inference_test tests/semantic_inference_test.cpp)

    add_test(NAME voxel4d_poc_smoke_test COMMAND voxel4d_poc)
    set_tests_properties(voxel4d_poc_smoke_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()
