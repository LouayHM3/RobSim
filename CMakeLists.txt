cmake_minimum_required(VERSION 3.24)
project(RoboSim VERSION 0.2.0 LANGUAGES CXX)

if(POLICY CMP0168)
  cmake_policy(SET CMP0168 NEW)
endif()

include(FetchContent)
include(CTest)
option(ROBOSIM_USE_SYSTEM_DEPS "Use installed yaml-cpp and nlohmann_json packages" OFF)
option(ROBOSIM_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)

if(ROBOSIM_USE_SYSTEM_DEPS)
  find_package(yaml-cpp 0.8 CONFIG REQUIRED)
  find_package(nlohmann_json 3.11 CONFIG REQUIRED)
else()
  # yaml-cpp 0.8 declares a pre-3.5 minimum; CMake 4 needs this policy floor.
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  endif()
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
    URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(yaml-cpp nlohmann_json)
endif()

add_library(robosim_core
  src/scenario.cpp src/components.cpp src/engine.cpp src/metrics.cpp src/reporting.cpp)
target_include_directories(robosim_core PUBLIC include)
target_compile_features(robosim_core PUBLIC cxx_std_17)
set_target_properties(robosim_core PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(robosim_core PUBLIC nlohmann_json::nlohmann_json PRIVATE yaml-cpp::yaml-cpp)
if(MINGW)
  target_link_options(robosim_core PUBLIC -static)
endif()
if(MSVC)
  target_compile_options(robosim_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(robosim_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()
if(ROBOSIM_SANITIZERS AND NOT MSVC)
  target_compile_options(robosim_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(robosim_core PUBLIC -fsanitize=address,undefined)
endif()

add_executable(robosim src/main.cpp)
target_link_libraries(robosim PRIVATE robosim_core)
install(TARGETS robosim RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  add_executable(robosim_tests tests/test_simulation.cpp)
  target_link_libraries(robosim_tests PRIVATE robosim_core)
  add_test(NAME simulation COMMAND robosim_tests "${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME verify_scenarios COMMAND robosim verify
    "${CMAKE_CURRENT_SOURCE_DIR}/scenarios" --output "${CMAKE_CURRENT_BINARY_DIR}/verification")
  add_test(NAME cli_help COMMAND robosim --help)
  add_test(NAME cli_invalid_repeats COMMAND robosim benchmark
    "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/nominal.yaml" --repeats 0)
  set_tests_properties(cli_invalid_repeats PROPERTIES WILL_FAIL TRUE)
  add_test(NAME cli_empty_directory COMMAND robosim verify "${CMAKE_CURRENT_BINARY_DIR}/empty")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/empty")
  set_tests_properties(cli_empty_directory PROPERTIES WILL_FAIL TRUE)
  add_test(NAME cli_contract COMMAND "${CMAKE_COMMAND}"
    "-DROBOSIM=$<TARGET_FILE:robosim>"
    "-DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/cli-artifacts"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_contract.cmake")
endif()
