set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.23)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(h743fpga C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv5-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv5-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m7 -mthumb)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

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

include_directories(
    Core/Inc 
    Drivers/STM32H7xx_HAL_Driver/Inc 
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy 
    Drivers/CMSIS/Device/ST/STM32H7xx/Include 
    Drivers/CMSIS/Include
    ../h743framework/Peripherals/
    ../h743framework/Drivers/CMSIS/DSP/Include/
    ../h743framework/SignalNet/Inc/
    ../h743framework/Utils/
)

add_definitions(
    -DUSE_HAL_DRIVER 
    -DSTM32H743xx
    -DARM_MATH_CM7
    -DSCREEN_USE_ADDT
    -DSTM32H743xx
)

# Read Makefile and extract sources with regex
file(STRINGS Makefile HAL_SOURCES REGEX "Drivers/STM32H7xx_HAL_Driver/Src/")
list(TRANSFORM HAL_SOURCES REPLACE "\\\\" "")
list(TRANSFORM HAL_SOURCES STRIP)
# message(STATUS "HAL_SOURCES: ${HAL_SOURCES}")

file(GLOB_RECURSE SOURCES 
    "Core/*.c" 
    "../h743framework/SignalNet/Src/*.c"
    "../h743framework/Utils/*.c"
    "../h743framework/Peripherals/*.c"
    startup_stm32h743xx.s
)
list(FILTER SOURCES EXCLUDE REGEX ".*Templates.*")
list(APPEND SOURCES ${HAL_SOURCES})

# message(STATUS "SOURCES: ${SOURCES}")

link_directories(
  ../h743framework/Drivers/CMSIS/DSP/Lib/ARM
  ../h743framework/Drivers/CMSIS/DSP/Lib/GCC
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H743VIHx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb)
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(-u _printf_float)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}.elf 
    m 
    c 
    nosys 
    arm_cortexM7lfdp_math
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")