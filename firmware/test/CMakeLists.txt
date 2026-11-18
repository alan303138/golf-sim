# Host-side unit tests for the portable firmware library (firmware/lib).
# Builds with the system C compiler; no pico-sdk needed.
#   cmake -S firmware/test -B build-test && cmake --build build-test && ctest --test-dir build-test
cmake_minimum_required(VERSION 3.13)
project(gs_firmware_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(GS_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../lib)
add_library(gs_portable STATIC
  ${GS_LIB_DIR}/gs_crc16.c
  ${GS_LIB_DIR}/gs_cobs.c
  ${GS_LIB_DIR}/gs_proto.c
  ${GS_LIB_DIR}/gs_sched.c
  ${GS_LIB_DIR}/gs_limiter.c
  ${GS_LIB_DIR}/gs_core.c
)
target_include_directories(gs_portable PUBLIC ${GS_LIB_DIR})
target_compile_options(gs_portable PRIVATE -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion)
target_link_libraries(gs_portable PUBLIC m)

option(GS_SANITIZE "Build tests with ASan/UBSan" ON)
if(GS_SANITIZE AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(gs_portable PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(gs_portable PUBLIC -fsanitize=address,undefined)
endif()

enable_testing()
foreach(t test_crc_cobs test_proto test_sched test_limiter test_core)
  add_executable(${t} ${t}.c)
  target_link_libraries(${t} gs_portable)
  target_compile_options(${t} PRIVATE -Wall -Wextra)
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# Test-vector tool used by the Python cross-check tests (gs_mcu/tests).
add_executable(gs_vectors gs_vectors.c)
target_link_libraries(gs_vectors gs_portable)
target_compile_options(gs_vectors PRIVATE -Wall -Wextra)
