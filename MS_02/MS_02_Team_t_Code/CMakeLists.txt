cmake_minimum_required(VERSION 3.13)

# Write build/compile_commands.json so the editor (clangd / IntelliSense)
# finds the Pico SDK headers instead of reporting 'pico/stdlib.h' not found.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Raspberry Pi Pico W
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Finds the Pico SDK (PICO_SDK_PATH, or the copy the VS Code Pico extension installs)
include(pico_sdk_import.cmake)

project(ms02_neural_calibrator C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ms02_neural_calibrator
    main.c
    servo.c
    fsr.c
    rgb_led.c
    motor.c
    mic.c
)

# Builds i2s_in.pio.h from the PIO program used by the mic driver
pico_generate_pio_header(ms02_neural_calibrator ${CMAKE_CURRENT_LIST_DIR}/i2s_in.pio)

target_include_directories(ms02_neural_calibrator PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(ms02_neural_calibrator
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_pio
    hardware_clocks
)

# printf goes to USB serial
pico_enable_stdio_usb(ms02_neural_calibrator 1)
pico_enable_stdio_uart(ms02_neural_calibrator 0)

# Produces the .uf2 file to drag onto the Pico
pico_add_extra_outputs(ms02_neural_calibrator)
