cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(parking_robot_profiles LANGUAGES C ASM)

if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_OBJCOPY OR NOT CMAKE_SIZE)
    message(FATAL_ERROR
        "Use CMake preset 'production' with the GNU Arm Embedded toolchain")
endif()

set(STM32_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(STM32_ARTIFACT_DIR "${CMAKE_BINARY_DIR}/artifacts")
file(MAKE_DIRECTORY "${STM32_ARTIFACT_DIR}")

set(STM32_APPLICATION_SOURCES
    Core/Src/main.c
    Core/Src/parking_robot_firmware.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f4xx.c
    Core/Startup/startup_stm32f401retx.s
)

set(STM32_HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

set(STM32_INCLUDE_DIRECTORIES
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

set(STM32_CPU_OPTIONS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

function(add_parking_robot_profile target_name artifact_name profile_value)
    add_executable(${target_name}
        ${STM32_APPLICATION_SOURCES}
        ${STM32_HAL_SOURCES}
    )
    set_target_properties(${target_name} PROPERTIES
        OUTPUT_NAME "${artifact_name}"
        SUFFIX ".elf"
        RUNTIME_OUTPUT_DIRECTORY "${STM32_ARTIFACT_DIR}"
        ADDITIONAL_CLEAN_FILES
            "${STM32_ARTIFACT_DIR}/${artifact_name}.bin;${STM32_ARTIFACT_DIR}/${artifact_name}.hex;${STM32_ARTIFACT_DIR}/${artifact_name}.map"
    )
    set_property(TARGET ${target_name} APPEND PROPERTY LINK_DEPENDS
        "${STM32_PROJECT_ROOT}/STM32F401RETX_FLASH.ld")

    target_include_directories(${target_name} PRIVATE
        ${STM32_INCLUDE_DIRECTORIES})
    if(ARM_NONE_EABI_NEWLIB_INCLUDE)
        target_include_directories(${target_name} SYSTEM PRIVATE
            "${ARM_NONE_EABI_NEWLIB_INCLUDE}/nano"
            "${ARM_NONE_EABI_NEWLIB_INCLUDE}")
    endif()
    target_compile_definitions(${target_name} PRIVATE
        USE_HAL_DRIVER
        STM32F401xE
        PARKING_ROBOT_PROFILE=${profile_value}
    )
    target_compile_options(${target_name} PRIVATE
        ${STM32_CPU_OPTIONS}
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
        $<$<COMPILE_LANGUAGE:C>:-Wall>
        $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
        $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
        $<$<COMPILE_LANGUAGE:C>:-fstack-usage>
        $<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:Release>>:-Os>
        $<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:Debug>>:-O0>
        $<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:Debug>>:-g3>
    )
    if(ARM_NONE_EABI_NEWLIB_PREFIX)
        # -B is multilib-aware: GCC selects thumb/v7e-m+fp/hard for Cortex-M4F.
        target_compile_options(${target_name} PRIVATE
            "-B${ARM_NONE_EABI_NEWLIB_PREFIX}/")
    endif()
    target_link_options(${target_name} PRIVATE
        ${STM32_CPU_OPTIONS}
        --specs=nano.specs
        --specs=nosys.specs
        "-T${STM32_PROJECT_ROOT}/STM32F401RETX_FLASH.ld"
        "-Wl,-Map=${STM32_ARTIFACT_DIR}/${artifact_name}.map"
        -Wl,--gc-sections
        -Wl,--print-memory-usage
    )
    if(ARM_NONE_EABI_NEWLIB_PREFIX)
        target_link_options(${target_name} PRIVATE
            "-B${ARM_NONE_EABI_NEWLIB_PREFIX}/")
    endif()
    target_link_libraries(${target_name} PRIVATE m)

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary
            "$<TARGET_FILE:${target_name}>"
            "${STM32_ARTIFACT_DIR}/${artifact_name}.bin"
        COMMAND "${CMAKE_OBJCOPY}" -O ihex
            "$<TARGET_FILE:${target_name}>"
            "${STM32_ARTIFACT_DIR}/${artifact_name}.hex"
        COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:${target_name}>"
        COMMENT "Generating ${artifact_name}.bin and ${artifact_name}.hex"
        VERBATIM
    )
endfunction()

# There is deliberately no generic/default parking_robot target.  Every
# flashable output carries an explicit physical profile in both target and
# filename.
add_parking_robot_profile(parking_robot_front parking_robot_front 1)
add_parking_robot_profile(parking_robot_rear parking_robot_rear 2)

add_custom_target(build_front_firmware DEPENDS parking_robot_front)
add_custom_target(build_rear_firmware DEPENDS parking_robot_rear)
