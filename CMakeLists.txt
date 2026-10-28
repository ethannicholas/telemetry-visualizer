cmake_minimum_required(VERSION 3.16)
project(telemetry_visualizer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SDL2: an installed one if there is (Homebrew, a distribution, a vcpkg prefix), otherwise
# built from source with the project. Windows on ARM64 has no prebuilt SDL2.
find_package(SDL2 CONFIG QUIET)
if(SDL2_FOUND)
  if(TARGET SDL2::SDL2)
    set(SDL_TARGET SDL2::SDL2)
  else()
    set(SDL_TARGET SDL2::SDL2-static)
  endif()
else()
  include(FetchContent)
  set(SDL_SHARED OFF CACHE BOOL "" FORCE)
  set(SDL_STATIC ON CACHE BOOL "" FORCE)
  set(SDL_TEST OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(SDL2
    URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.9/SDL2-2.30.9.tar.gz)
  FetchContent_MakeAvailable(SDL2)
  set(SDL_TARGET SDL2::SDL2-static)
endif()

find_package(OpenGL REQUIRED)

add_executable(telemetry-visualizer WIN32
  src/main.cpp src/packet.cpp src/udp.cpp)
target_include_directories(telemetry-visualizer PRIVATE third_party)
target_link_libraries(telemetry-visualizer PRIVATE ${SDL_TARGET} OpenGL::GL)
if(TARGET SDL2::SDL2main)
  target_link_libraries(telemetry-visualizer PRIVATE SDL2::SDL2main)
endif()
if(WIN32)
  target_link_libraries(telemetry-visualizer PRIVATE ws2_32)
  target_compile_definitions(telemetry-visualizer PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()
if(APPLE)
  target_compile_definitions(telemetry-visualizer PRIVATE GL_SILENCE_DEPRECATION)
  # SDL2's own installer puts a framework in /Library/Frameworks that loads by @rpath.
  if(SDL2_DIR MATCHES "^(.*)/SDL2\\.framework/")
    set_target_properties(telemetry-visualizer PROPERTIES BUILD_RPATH "${CMAKE_MATCH_1}")
  endif()
endif()
