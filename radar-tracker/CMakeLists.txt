cmake_minimum_required(VERSION 3.16)
project(radar_tracker VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to an optimized build so the timing numbers are meaningful.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Everything except main.cpp goes into a library so the tests can link it too.
add_library(tracker_core
  src/Simulator.cpp
  src/KalmanFilter.cpp
  src/Hungarian.cpp
  src/Tracker.cpp
)
target_include_directories(tracker_core PUBLIC include)

if(MSVC)
  target_compile_options(tracker_core PUBLIC /W4)
else()
  target_compile_options(tracker_core PUBLIC -Wall -Wextra -Wpedantic)
endif()

add_executable(radar_tracker src/main.cpp)
target_link_libraries(radar_tracker PRIVATE tracker_core)

enable_testing()
add_executable(tracker_tests tests/test_main.cpp)
target_link_libraries(tracker_tests PRIVATE tracker_core)
add_test(NAME tracker_tests COMMAND tracker_tests)
