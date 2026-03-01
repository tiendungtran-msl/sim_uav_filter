cmake_minimum_required(VERSION 3.16)
project(ekf2_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Compile flags ─────────────────────────────────────────────────────────
# -fno-exceptions and -fno-rtti are required for NuttX compatibility.
# For host testing, they are optional but enforced here to remain portable.
add_compile_options(
    -Wall -Wextra -Wpedantic
    -fno-exceptions
    -fno-rtti
    -O2
)

# ── Library ───────────────────────────────────────────────────────────────
add_library(ekf2_core STATIC
    src/ekf.cpp
    src/imu_update.cpp
    src/gps_update.cpp
    src/covariance.cpp
    src/buffer.cpp
)
target_include_directories(ekf2_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── Host test executable (CSV replay) ─────────────────────────────────────
add_executable(ekf2_replay src/replay_main.cpp)
target_link_libraries(ekf2_replay ekf2_core)

# ── Cross-compile for NuttX (arm-none-eabi) ───────────────────────────────
# To cross-compile, pass:
#   -DCMAKE_TOOLCHAIN_FILE=../cmake/arm_none_eabi.cmake
# The library target produces a .a suitable for linking into PX4 modules.
