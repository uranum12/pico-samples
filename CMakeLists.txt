cmake_minimum_required(VERSION 3.12)

include(cmake/pico_sdk_import.cmake)

project(pico_samples C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(blink src/blink.c)
target_link_libraries(blink pico_stdlib hardware_gpio)
pico_add_extra_outputs(blink)

add_executable(hello_world src/hello_world.c)
target_link_libraries(hello_world pico_stdlib)
pico_enable_stdio_usb(hello_world 1)
pico_enable_stdio_uart(hello_world 1)
pico_add_extra_outputs(hello_world)

add_executable(bme280 src/bme280.c)
target_link_libraries(bme280 pico_stdlib hardware_gpio hardware_spi)
pico_enable_stdio_usb(bme280 1)
pico_enable_stdio_uart(bme280 1)
pico_add_extra_outputs(bme280)

add_executable(bme280_i2c src/bme280_i2c.c)
target_link_libraries(bme280_i2c pico_stdlib hardware_gpio hardware_i2c)
pico_enable_stdio_usb(bme280_i2c 1)
pico_enable_stdio_uart(bme280_i2c 1)
pico_add_extra_outputs(bme280_i2c)

