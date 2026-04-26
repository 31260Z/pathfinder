cmake_minimum_required(VERSION 3.20)
project(pathfinder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json for clangd / IDE tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Treat warnings as errors and turn most of them on. Library code must compile clean.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror)
endif()

# ──────────────────────────────────────────────────────────────────────────
# Library — header-only at this stage. Modules add their .cpp files (if any)
# to PATHFINDER_SOURCES via target_sources() in their own CMakeLists.txt.
# ──────────────────────────────────────────────────────────────────────────
add_library(pathfinder INTERFACE)
target_include_directories(pathfinder INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(pathfinder INTERFACE cxx_std_20)

# Module sources will be added here by per-module CMakeLists if they exist.
foreach(module IN ITEMS geometry controllers odometry calibration autonomous chassis driving sensors runtime)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${module}/CMakeLists.txt)
    add_subdirectory(src/${module})
  endif()
endforeach()

# ──────────────────────────────────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
