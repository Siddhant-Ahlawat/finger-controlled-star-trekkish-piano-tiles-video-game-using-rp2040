cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(blink_new C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(mandelbrot-fixvfloat)

# must match with pio filename and executable name from above
pico_generate_pio_header(mandelbrot-fixvfloat ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(mandelbrot-fixvfloat ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(mandelbrot-fixvfloat ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

pico_enable_stdio_usb(mandelbrot-fixvfloat 1)
pico_enable_stdio_uart(mandelbrot-fixvfloat 0)

# must match with executable name and source file names
target_sources(mandelbrot-fixvfloat PRIVATE mandelbrot_fixvfloat.c vga_graphics.c registers.h)

# must match with executable name
target_link_libraries(mandelbrot-fixvfloat PRIVATE pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_spi hardware_sync hardware_pio hardware_dma hardware_adc)

# must match with executable name
pico_add_extra_outputs(mandelbrot-fixvfloat)
