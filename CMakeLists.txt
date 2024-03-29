#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_VERBOSE_MAKEFILE on)
set(INTERMEDIATE off)

# project settings
project(MonitorCPP C CXX ASM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=Cortex-m3 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options(--save-temps)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

#include_directories()
include_directories(
        Inc
        include
        Drivers/CMSIS/Include
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        ../../Arduino/monitor/src
        ../LatheCPP/include
)

add_definitions(
        -DDEBUG
        -DSTM32F103xB
        -DSTM32F1
        -DSTM32MON
        -DUSE_FUL_LL_DRIVER
        -DARDUINO_BLUEPILL_F103C8
)

#file(GLOB_RECURSE SOURCES )
file(GLOB_RECURSE SOURCES "Startup/*.*" "Drivers/*.*")
set(SOURCES1
Src/startup_stm32f103xb.S
Src/adc.c
Src/dma.c
Src/gpio.c
Src/i2c.c
Src/main.c
Src/rtc.c
Src/spi.c
Src/stm32f1xx_hal_msp.c
Src/stm32f1xx_it.c
Src/syscalls.c
Src/system_stm32f1xx.c
Src/sysmem.c
Src/tim.c
Src/usart.c
Src/current.cpp
Src/lclcmd.cpp
Src/monitorLoop.cpp
Src/serialio.cpp
../../Arduino/monitor/src/max31856.cpp
../../Arduino/monitor/src/max31865.cpp
../LatheCPP/lathe_src/getSP.S
../LatheCPP/lathe_src/lcd.cpp
../LatheCPP/lathe_src/i2cx.cpp
../LatheCPP/lathe_src/spix.cpp
../LatheCPP/lathe_src/stm32Info.cpp)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103CBTX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=Cortex-m3 -mthumb -mthumb-interwork)
add_link_options(-Wl,--no-warn-rwx-segments)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES1} ${SOURCES} ${LINKER_SCRIPT})

if(INTERMEDIATE)
foreach(SRC_ IN LISTS SOURCES1)
    get_filename_component(SRC_BASENAME_ ${SRC_} NAME_WE)
    set_source_files_properties(${SRC_} PROPERTIES COMPILE_OPTIONS "-Wa,-adhlns=${SRC_BASENAME_}.lst")
    get_source_file_property(SRC_PROP_ ${SRC_} COMPILE_OPTIONS)
    message(STATUS "${SRC_}: ${SRC_PROP_}")
endforeach()
endif()

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
