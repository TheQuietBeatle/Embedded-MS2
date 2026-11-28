cmake_minimum_required(VERSION 3.13)    # oldest CMake version that can build this

# Write build/compile_commands.json so the editor (clangd / IntelliSense)
# finds the Pico SDK headers instead of reporting 'pico/stdlib.h' not found.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # see the comment above

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
set(PICO_BOARD pico_w CACHE STRING "Board type")        # the build target is a Pico W (needed for Wi-Fi)

# Finds the Pico SDK (PICO_SDK_PATH, or the copy the VS Code Pico extension installs)
include(pico_sdk_import.cmake)          # must come before project()

project(ms02_neural_calibrator C CXX ASM)               # project name; C, C++ and assembly are used
set(CMAKE_C_STANDARD 11)                # C11
set(CMAKE_CXX_STANDARD 17)              # C++17 (parts of the SDK are C++)

pico_sdk_init()                         # set up the Pico SDK

add_executable(ms02_neural_calibrator   # first program: the MS2 driver test
    main.c                              # the test sequence
    servo.c                             # arm servo driver
    gripper.c                           # gripper driver
    fsr.c                               # force sensor driver
    leds.c                              # status LED driver
    motor.c                             # reward motor driver
    mic.c                               # microphone driver
)                                       # end of source list

target_include_directories(ms02_neural_calibrator PRIVATE ${CMAKE_CURRENT_LIST_DIR})  # look for headers in this folder

target_link_libraries(ms02_neural_calibrator            # SDK libraries the driver test uses
    pico_stdlib                         # GPIO, timers, stdio
    hardware_pwm                        # servos, LEDs, motor speed
    hardware_adc                        # force sensor and microphone
    hardware_clocks                     # clock_get_hz() for the PWM dividers
)                                       # end of library list

# printf goes to USB serial
pico_enable_stdio_usb(ms02_neural_calibrator 1)         # USB serial on
pico_enable_stdio_uart(ms02_neural_calibrator 0)        # UART serial off

# Produces the .uf2 file to drag onto the Pico
pico_add_extra_outputs(ms02_neural_calibrator)          # .uf2, .bin, .hex and others

# ---------------------------------------------------------------------------
# Second program: web page that controls the arm and the reward motor.
# Copy wifi_config.example.h to wifi_config.h and set your Wi-Fi first.
# Builds ms02_web_control.uf2 next to the driver test.
add_executable(ms02_web_control         # second program: the web control
    web_control.c                       # web server, Wi-Fi and main loop
    dhcp_server.c                       # IP addresses for the Pico's own network
    servo.c                             # arm servo driver
    gripper.c                           # gripper driver
    motor.c                             # reward motor driver
    leds.c                              # status LED driver
    fsr.c                               # force sensor driver
    mic.c                               # microphone driver
)                                       # end of source list

target_include_directories(ms02_web_control PRIVATE ${CMAKE_CURRENT_LIST_DIR})  # look for headers in this folder (lwipopts.h too)

target_link_libraries(ms02_web_control  # SDK libraries the web control uses
    pico_stdlib                         # GPIO, timers, stdio
    pico_cyw43_arch_lwip_threadsafe_background          # Wi-Fi chip + lwIP, run from background interrupts
    hardware_pwm                        # servos, LEDs, motor speed
    hardware_adc                        # force sensor and microphone
    hardware_clocks                     # clock_get_hz() for the PWM dividers
)                                       # end of library list

pico_enable_stdio_usb(ms02_web_control 1)               # USB serial on
pico_enable_stdio_uart(ms02_web_control 0)              # UART serial off
pico_add_extra_outputs(ms02_web_control)                # .uf2, .bin, .hex and others
