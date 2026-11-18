# Timing-MCU firmware for the golf launch monitor (Raspberry Pi Pico / RP2040).
#   export PICO_SDK_PATH=/path/to/pico-sdk   (SDK 2.x, with submodules)
#   cmake -S firmware -B build-fw && cmake --build build-fw
# Output: build-fw/gs_timing.uf2
cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico)
set(PICO_PLATFORM rp2040)
include(pico_sdk_import.cmake)
project(gs_timing VERSION 1.0.0 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)

# USB identity (descriptors are compiled inside the SDK, so define before pico_sdk_init)
add_compile_definitions(
  USBD_PRODUCT=\"GolfSim-Timing\"
  USBD_MANUFACTURER=\"golf-sim\"
)
pico_sdk_init()

add_executable(gs_timing
  src/main.c
  src/hw.c
  lib/gs_crc16.c
  lib/gs_cobs.c
  lib/gs_proto.c
  lib/gs_sched.c
  lib/gs_limiter.c
  lib/gs_core.c
)
pico_generate_pio_header(gs_timing ${CMAKE_CURRENT_LIST_DIR}/src/gs_wave.pio)
target_include_directories(gs_timing PRIVATE lib src)
target_compile_options(gs_timing PRIVATE -Wall -Wextra)
target_compile_definitions(gs_timing PRIVATE
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=50000
  PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
  PICO_STDIO_DEFAULT_CRLF=0
  PICO_STDIO_ENABLE_CRLF_SUPPORT=0
)
target_link_libraries(gs_timing
  pico_stdlib hardware_pio hardware_dma hardware_adc hardware_irq hardware_timer
  hardware_watchdog hardware_clocks hardware_sync pico_unique_id pico_bootrom)
pico_enable_stdio_usb(gs_timing 1)
pico_enable_stdio_uart(gs_timing 0)
pico_add_extra_outputs(gs_timing)
