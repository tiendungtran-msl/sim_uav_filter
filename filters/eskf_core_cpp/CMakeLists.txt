# ==============================================================================
# CMakeLists.txt — ESKF Core C++ Build System
#
# Target chính:
#   eskf_core     — thư viện tĩnh chứa toàn bộ ESKF logic
#   eskf_replay   — tool offline replay
#   eskf_bench    — tool benchmark
#
# Cách build:
#   cd filters/eskf_core_cpp
#   mkdir -p build && cd build
#   cmake .. -DCMAKE_BUILD_TYPE=Release
#   make -j$(nproc)
#
# Cách chạy:
#   ./eskf_replay ../../data/examples/sensor_log.csv
#   ./eskf_bench  ../../data/examples/sensor_log.csv
# ==============================================================================

cmake_minimum_required(VERSION 3.16)
project(eskf_core_cpp VERSION 1.0 LANGUAGES CXX)

# C++17 — dùng tối thiểu (std::array, constexpr)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Tối ưu hóa cho release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Flags
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")

# ---- Include paths ----
include_directories(${CMAKE_SOURCE_DIR}/include)

# ---- ESKF Core static library ----
add_library(eskf_core STATIC
    src/eskf.cpp
    src/imu_propagation.cpp
    src/gps_update.cpp
    src/mag_update.cpp
    src/baro_update.cpp
    src/numerics.cpp
)

target_include_directories(eskf_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# ---- Replay tool ----
add_executable(eskf_replay
    tools/replay_main.cpp
)
target_link_libraries(eskf_replay PRIVATE eskf_core)

# ---- Benchmark tool ----
add_executable(eskf_bench
    tools/bench_main.cpp
)
target_link_libraries(eskf_bench PRIVATE eskf_core)

# ---- Tests ----
add_executable(test_math tests/test_math.cpp)
target_link_libraries(test_math PRIVATE eskf_core)

add_executable(test_buffer tests/test_buffer.cpp)
target_link_libraries(test_buffer PRIVATE eskf_core)

add_executable(test_consistency tests/test_consistency.cpp)
target_link_libraries(test_consistency PRIVATE eskf_core)
