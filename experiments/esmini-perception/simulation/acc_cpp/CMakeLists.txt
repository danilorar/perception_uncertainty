# Author: Zhuo Ma
# Portable experiment core; only acc_sim depends on the esmini shared library.
cmake_minimum_required(VERSION 3.21)
project(accsim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(ACCSIM_BUILD_RUNNER "Build the esmini-backed acc_sim executable" ON)
set(ESMINI_HOME "$ENV{ESMINI_HOME}" CACHE PATH "Root of an esmini v3.8.1 distribution")
set(ACCSIM_TEST_SCENARIO "${PROJECT_SOURCE_DIR}/../../Data/C_original_1549178.xosc"
    CACHE FILEPATH "Original scenario used by the scenario preparation test")

add_library(accsim_xml STATIC third_party/pugixml/pugixml.cpp)
target_include_directories(accsim_xml PUBLIC third_party/pugixml)
target_compile_definitions(accsim_xml PUBLIC pugi=accsim_xml)

add_library(accsim_core STATIC
    src/scenario.cpp src/esmini_acc.cpp src/esmini_aeb.cpp src/vehicle.cpp src/perception.cpp src/metrics.cpp)
add_library(accsim::core ALIAS accsim_core)
target_include_directories(accsim_core PUBLIC include)
target_link_libraries(accsim_core PRIVATE accsim_xml)
if(MSVC)
    target_compile_options(accsim_core PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(accsim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(ACCSIM_BUILD_RUNNER)
    if(NOT ESMINI_HOME)
        message(FATAL_ERROR
            "Set ESMINI_HOME to an esmini v3.8.1 distribution, or use "
            "-DACCSIM_BUILD_RUNNER=OFF to build and test the portable core alone.")
    endif()
    cmake_path(ABSOLUTE_PATH ESMINI_HOME NORMALIZE)
    if(NOT EXISTS "${ESMINI_HOME}/version.txt")
        message(FATAL_ERROR "Missing ${ESMINI_HOME}/version.txt; a versioned esmini distribution is required.")
    endif()
    file(READ "${ESMINI_HOME}/version.txt" ACCSIM_ESMINI_VERSION)
    if(NOT ACCSIM_ESMINI_VERSION MATCHES "ESMINI_GIT_TAG=\"v3\\.8\\.1\"")
        message(FATAL_ERROR "This experiment pins esmini v3.8.1; version.txt does not identify that tag.")
    endif()
    find_path(ESMINI_INCLUDE_DIR esminiLib.hpp
        PATHS "${ESMINI_HOME}/EnvironmentSimulator/Libraries/esminiLib"
              "${ESMINI_HOME}/include" NO_DEFAULT_PATH REQUIRED)
    add_library(esmini::library SHARED IMPORTED GLOBAL)
    if(WIN32)
        find_file(ESMINI_DLL NAMES esminiLib.dll libesminiLib.dll
            PATHS "${ESMINI_HOME}/bin" "${ESMINI_HOME}/lib" NO_DEFAULT_PATH REQUIRED)
        find_library(ESMINI_IMPORT_LIBRARY NAMES esminiLib libesminiLib
            PATHS "${ESMINI_HOME}/bin" "${ESMINI_HOME}/lib" NO_DEFAULT_PATH REQUIRED)
        set_target_properties(esmini::library PROPERTIES
            IMPORTED_LOCATION "${ESMINI_DLL}" IMPORTED_IMPLIB "${ESMINI_IMPORT_LIBRARY}")
    else()
        find_library(ESMINI_LIBRARY NAMES esminiLib
            PATHS "${ESMINI_HOME}/bin" "${ESMINI_HOME}/lib" NO_DEFAULT_PATH REQUIRED)
        set_target_properties(esmini::library PROPERTIES IMPORTED_LOCATION "${ESMINI_LIBRARY}")
    endif()
    set_target_properties(esmini::library PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ESMINI_INCLUDE_DIR}")
    # The distribution must include our bridge in addition to standard SE_* APIs.
    if(NOT EXISTS "${ESMINI_HOME}/accsim_bridge.json")
        message(FATAL_ERROR "Build the native ACC bridge first: python tools/build_native.py")
    endif()
    file(READ "${ESMINI_HOME}/accsim_bridge.json" ACCSIM_BRIDGE_MANIFEST)
    string(JSON ACCSIM_BRIDGE_ABI ERROR_VARIABLE ACCSIM_BRIDGE_ERROR GET "${ACCSIM_BRIDGE_MANIFEST}" abi_version)
    if(ACCSIM_BRIDGE_ERROR OR NOT ACCSIM_BRIDGE_ABI EQUAL 2)
        message(FATAL_ERROR "Native ACC/AEB bridge ABI 2 required; rebuild with python tools/build_native.py")
    endif()
    target_link_libraries(accsim_core PUBLIC esmini::library)
    add_executable(acc_sim src/main.cpp src/esmini_adapter.cpp)
    target_link_libraries(acc_sim PRIVATE accsim::core esmini::library)
    if(MSVC)
        target_compile_options(acc_sim PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(acc_sim PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(WIN32)
        # Windows resolves dependent DLLs beside the executable. Preserve the
        # distribution's dependencies without asking users to change global PATH.
        file(GLOB ACCSIM_ENGINE_DLLS CONFIGURE_DEPENDS "${ESMINI_HOME}/bin/*.dll" "${ESMINI_HOME}/lib/*.dll")
        add_custom_command(TARGET acc_sim POST_BUILD COMMAND "${CMAKE_COMMAND}" -E
            copy_if_different ${ACCSIM_ENGINE_DLLS} "$<TARGET_FILE_DIR:acc_sim>"
            COMMENT "Copying esmini runtime DLLs" VERBATIM)
    else()
        # These are local build paths, never checked-in machine-specific paths.
        set_target_properties(acc_sim PROPERTIES
            BUILD_RPATH "${ESMINI_HOME}/bin;${ESMINI_HOME}/lib")
    endif()
endif()

include(CTest)
if(BUILD_TESTING)
    foreach(test_name scenario_tests core_tests markov_tests)
        add_executable(${test_name} tests/${test_name}.cpp)
        target_link_libraries(${test_name} PRIVATE accsim::core)
    endforeach()
    add_test(NAME perception_vehicle_metrics COMMAND core_tests)
    add_test(NAME markov_temporal_dropout COMMAND markov_tests)
    target_link_libraries(scenario_tests PRIVATE accsim_xml)
    if(EXISTS "${ACCSIM_TEST_SCENARIO}")
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-data")
        add_test(NAME scenario_preparation COMMAND scenario_tests
            "${ACCSIM_TEST_SCENARIO}" "${CMAKE_CURRENT_BINARY_DIR}/test-data/generated.xosc")
        # Keep the original fixture and cover both other supplied rear-end cases.
        # 1554431 also guards against restoring an accumulated-lateral-offset cap.
        foreach(scenario_id 1554254 1554431)
            set(scenario_input "${PROJECT_SOURCE_DIR}/../../Data/C_original_${scenario_id}.xosc")
            if(EXISTS "${scenario_input}")
                file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-data/${scenario_id}")
                add_test(NAME scenario_preparation_${scenario_id} COMMAND scenario_tests
                    "${scenario_input}" "${CMAKE_CURRENT_BINARY_DIR}/test-data/${scenario_id}/generated.xosc")
            endif()
        endforeach()
        target_compile_definitions(scenario_tests PRIVATE pugi=accsim_xml)
        if(ACCSIM_BUILD_RUNNER)
            add_executable(acc_tests tests/acc_tests.cpp src/esmini_adapter.cpp)
            target_link_libraries(acc_tests PRIVATE accsim::core)
            add_test(NAME acc_controller COMMAND acc_tests "${ACCSIM_TEST_SCENARIO}"
                "${CMAKE_CURRENT_BINARY_DIR}/test-data/acc-test")
            add_executable(aeb_tests tests/aeb_tests.cpp src/esmini_adapter.cpp)
            target_link_libraries(aeb_tests PRIVATE accsim::core)
            add_test(NAME native_aeb_component COMMAND aeb_tests "${ACCSIM_TEST_SCENARIO}"
                "${CMAKE_CURRENT_BINARY_DIR}/test-data/aeb-test")
        endif()
    else()
        message(STATUS "Scenario preparation test not registered: set ACCSIM_TEST_SCENARIO to an existing original scenario.")
    endif()
endif()
