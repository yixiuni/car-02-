cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32-car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/app.c
    Core/Src/board_c10b.c
    Core/Src/hiwonder_gray.c
    Core/Src/gray_protocol.c
    Core/Src/mpu6050.c
    Core/Src/car_control.c
    Core/Src/motion_control.c
    Core/Src/encoder.c
    Core/Src/imu_filter.c
    Core/Src/motor_test.c
    Core/Src/motor_test_port.c
    Core/Src/ground_test.c
    Core/Src/task1_test.c
    Core/Src/oled.c
    Core/Src/oled_ui.c
    Core/Src/course_mode.c
    Core/Src/gray_analog.c
    Core/Src/gray_learn.c
)

option(CAR_MOTOR_TEST_BUILD "Build one-shot raised-wheel diagnostic firmware" OFF)
option(CAR_GROUND_TEST_BUILD "Build isolated 200/500/1000 mm ground diagnostic" OFF)
option(CAR_TASK1_TEST_BUILD "Build one-shot real A-to-B controller test" OFF)
option(CAR_ARC_TEST_BUILD "Build one-shot A-B-C semicircle diagnostic" OFF)
option(CAR_LAP_TEST_BUILD "Build one-shot full task 2 lap diagnostic" OFF)
option(CAR_CROSS_TEST_BUILD "Build one-shot task 3 crossing lap diagnostic" OFF)
option(CAR_FOUR_TEST_BUILD "Build one-shot task4 four crossing laps" OFF)
option(CAR_DUAL_MODE_BUILD "Build selectable oval/cross routes and lap count" OFF)
option(CAR_TIME_TRIAL_BUILD "Build experimental T301 speed-planned dual mode" OFF)
option(CAR_TIME_TRIAL_STAGE2 "Build experimental T302 straight500 arc430 entry370" OFF)
option(CAR_TIME_TRIAL_STAGE3 "Build experimental T305 straight550 arc470 entry370, fading bounded entry recovery" OFF)
option(CAR_TIME_TRIAL_C_ENTRY "Build T306 C-entry experiment on stage2 speeds" OFF)
if(CAR_TIME_TRIAL_C_ENTRY)
    set(CAR_TIME_TRIAL_STAGE2 ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_TIME_TRIAL_C_ENTRY=1)
endif()
if(CAR_TIME_TRIAL_STAGE2 AND CAR_TIME_TRIAL_STAGE3)
    message(FATAL_ERROR "Choose one time-trial stage")
endif()
if(CAR_TIME_TRIAL_STAGE3)
    set(CAR_TIME_TRIAL_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_TIME_TRIAL_STAGE3=1)
endif()
if(CAR_TIME_TRIAL_STAGE2)
    set(CAR_TIME_TRIAL_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_TIME_TRIAL_STAGE2=1)
endif()
if(CAR_TIME_TRIAL_BUILD)
    set(CAR_DUAL_MODE_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_TIME_TRIAL_BUILD=1)
endif()
if(CAR_DUAL_MODE_BUILD)
    if(CAR_ARC_TEST_BUILD OR CAR_LAP_TEST_BUILD OR CAR_CROSS_TEST_BUILD OR CAR_FOUR_TEST_BUILD OR CAR_TASK1_TEST_BUILD)
        message(FATAL_ERROR "Choose dual mode or one course diagnostic")
    endif()
    set(CAR_TASK1_TEST_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_DUAL_MODE_BUILD=1)
endif()
if(CAR_FOUR_TEST_BUILD)
    if(CAR_ARC_TEST_BUILD OR CAR_LAP_TEST_BUILD OR CAR_CROSS_TEST_BUILD OR CAR_TASK1_TEST_BUILD)
        message(FATAL_ERROR "Choose one course diagnostic")
    endif()
    set(CAR_TASK1_TEST_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_FOUR_TEST_BUILD=1)
endif()
if(CAR_CROSS_TEST_BUILD)
    if(CAR_ARC_TEST_BUILD OR CAR_LAP_TEST_BUILD OR CAR_TASK1_TEST_BUILD)
        message(FATAL_ERROR "Choose one course diagnostic")
    endif()
    set(CAR_TASK1_TEST_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_CROSS_TEST_BUILD=1)
endif()
if(CAR_LAP_TEST_BUILD)
    if(CAR_ARC_TEST_BUILD OR CAR_TASK1_TEST_BUILD)
        message(FATAL_ERROR "Choose one course diagnostic")
    endif()
    set(CAR_TASK1_TEST_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_LAP_TEST_BUILD=1)
endif()
if(CAR_ARC_TEST_BUILD)
    if(CAR_TASK1_TEST_BUILD)
        message(FATAL_ERROR "Choose arc test or task 1 test, not both")
    endif()
    set(CAR_TASK1_TEST_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_ARC_TEST_BUILD=1)
endif()
option(CAR_GRAY_LEARN_BUILD "Build no-motion analog grayscale capture" OFF)
if(CAR_GRAY_LEARN_BUILD)
    if(CAR_MOTOR_TEST_BUILD OR CAR_GROUND_TEST_BUILD OR CAR_TASK1_TEST_BUILD)
        message(FATAL_ERROR "Gray learning cannot include motion diagnostics")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_GRAY_LEARN_BUILD=1)
endif()
if(CAR_GROUND_TEST_BUILD)
    set(CAR_MOTOR_TEST_BUILD ON)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_GROUND_TEST_BUILD=1)
endif()
if(CAR_TASK1_TEST_BUILD)
    if(CAR_MOTOR_TEST_BUILD)
        message(FATAL_ERROR "Task 1 and motor diagnostics cannot be combined")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_TASK1_TEST_BUILD=1)
endif()
option(CAR_MOTOR_SLOW_DECAY "AT8236 slow-decay PWM (validated raised-wheel default)" ON)
if(CAR_MOTOR_TEST_BUILD)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAR_MOTOR_TEST_BUILD=1)
endif()
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    CAR_MOTOR_SLOW_DECAY=$<BOOL:${CAR_MOTOR_SLOW_DECAY}>)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate both common flashing formats beside the ELF file.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating HEX and BIN firmware"
    VERBATIM
)
