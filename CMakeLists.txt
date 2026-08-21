cmake_minimum_required(VERSION 3.25...4.2 FATAL_ERROR)

project(v8patch VERSION 0.1.0 LANGUAGES CXX)

if (NOT WIN32)
  message(FATAL_ERROR "v8patch targets the Win32 Roblox client")
endif()

if (NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "v8patch must be built 32-bit; configure with a x86 toolchain")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(V8PATCH_MINHOOK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/minhook"
    CACHE PATH "Path to the MinHook checkout")
set(V8PATCH_BOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/box3d"
    CACHE PATH "Path to the Box3D checkout")

foreach(dependency MINHOOK BOX3D)
  if (NOT EXISTS "${V8PATCH_${dependency}_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "${dependency} not found at ${V8PATCH_${dependency}_DIR}. "
      "Run: git submodule update --init --recursive")
  endif()
endforeach()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(MINHOOK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${V8PATCH_MINHOOK_DIR}" "${CMAKE_BINARY_DIR}/minhook" EXCLUDE_FROM_ALL)

set(BOX3D_VALIDATE OFF CACHE BOOL "" FORCE)
add_subdirectory("${V8PATCH_BOX3D_DIR}" "${CMAKE_BINARY_DIR}/box3d" EXCLUDE_FROM_ALL)

set(V8PATCH_RESTITUTION_ITERATIONS "4" CACHE STRING
    "Gauss-Seidel sweeps over a manifold's points in box3d's restitution stage")
target_compile_definitions(box3d PRIVATE B3_RESTITUTION_ITERATIONS=${V8PATCH_RESTITUTION_ITERATIONS})

add_library(v8patch SHARED
  src/config.cpp
  src/frame.cpp
  src/dllmain.cpp
  src/hook.cpp
  src/log.cpp
  src/patch.cpp
  src/paths.cpp
  src/target.cpp
  src/patches/framerate.cpp
  src/patches/harness.cpp
  src/patches/physics.cpp
  src/proxy/dbghelp.cpp
  src/rbx/constants.cpp
  src/rbx/dynamics.cpp
  src/rbx/engine.cpp
  src/sim/clumpstage.cpp
  src/sim/connectors.cpp
  src/sim/explosion.cpp
  src/sim/kernel.cpp
  src/sim/simbody.cpp
  src/sim/world.cpp
)

target_include_directories(v8patch PRIVATE src)
target_link_libraries(v8patch PRIVATE minhook box3d)

target_compile_definitions(v8patch PRIVATE
  WIN32_LEAN_AND_MEAN
  NOMINMAX
  UNICODE
  _UNICODE
  V8PATCH_VERSION="${PROJECT_VERSION}"
)

if (MSVC)
  target_compile_options(v8patch PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus /EHsc)
  target_link_options(v8patch PRIVATE /OPT:REF /OPT:ICF)
endif()

set_target_properties(v8patch PROPERTIES
  OUTPUT_NAME dbghelp
  PREFIX ""
)

enable_testing()

add_executable(v8patch_tests
  tests/main.cpp
  tests/clumps.cpp
  tests/constants.cpp
  tests/contactlaw.cpp
  tests/contacts.cpp
  tests/hashing.cpp
  tests/frame.cpp
  tests/stepping.cpp
  tests/winding.cpp
  src/frame.cpp
  src/rbx/constants.cpp
)

target_include_directories(v8patch_tests PRIVATE src tests)
target_link_libraries(v8patch_tests PRIVATE box3d)
target_compile_definitions(v8patch_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

if (MSVC)
  target_compile_options(v8patch_tests PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus /EHsc)
endif()

foreach(suite clumps constants contactlaw contacts hashing stepping winding)
  add_test(NAME ${suite} COMMAND v8patch_tests ${suite})
endforeach()
