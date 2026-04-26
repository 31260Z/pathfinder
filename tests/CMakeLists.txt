include(FetchContent)

# doctest v2.4.x ships with cmake_minimum_required(VERSION 3.0); CMake 4.x removed
# back-compat for < 3.5. This shim tells CMake to apply 3.5-era policies inside
# the fetched dependency without modifying its source.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.11
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(doctest)

# Helper to register a test executable. Each module's CMakeLists.txt calls this.
function(pathfinder_add_test target_name)
  add_executable(${target_name} ${ARGN})
  target_link_libraries(${target_name} PRIVATE
    pathfinder
    doctest::doctest_with_main
  )
  add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()

# Per-module test subdirectories. Each guarded by an if(EXISTS ...) so agents
# can populate them independently without merge conflicts.
foreach(module IN ITEMS geometry controllers odometry calibration autonomous chassis driving sensors runtime)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${module}/CMakeLists.txt)
    add_subdirectory(${module})
  endif()
endforeach()
