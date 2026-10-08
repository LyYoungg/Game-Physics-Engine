cmake_minimum_required(VERSION 3.16)

project(inertia3d LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Link the MSVC runtime statically so sandbox.exe runs on a PC without the
# Visual C++ redistributable installed.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

option(INERTIA_BUILD_APP "Build the raylib sandbox (downloads raylib)" ON)

# ---- Engine: a static library with no dependencies ------------------------
add_library(inertia STATIC
  engine/src/collision.cpp
  engine/src/world.cpp)
target_include_directories(inertia PUBLIC engine/include)
if(MSVC)
  target_compile_options(inertia PRIVATE /W4)
else()
  target_compile_options(inertia PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- Tests -----------------------------------------------------------------
enable_testing()
add_executable(inertia_tests tests/test_physics.cpp)
target_link_libraries(inertia_tests PRIVATE inertia)
add_test(NAME physics COMMAND inertia_tests)

# ---- Sandbox ---------------------------------------------------------------
if(INERTIA_BUILD_APP)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(raylib)

  add_executable(sandbox app/main.cpp)
  target_link_libraries(sandbox PRIVATE inertia raylib)
  if(MSVC)
    # Start from main() without opening a console window.
    target_link_options(sandbox PRIVATE /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup)
  elseif(MINGW)
    target_link_options(sandbox PRIVATE -mwindows -static)
  endif()
endif()
