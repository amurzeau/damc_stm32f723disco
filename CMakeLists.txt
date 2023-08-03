cmake_minimum_required(VERSION 3.20)

project("damc_stm32f723disco" C CXX ASM)

include(cmake/st-project.cmake)

set(CMAKE_CXX_STANDARD 17)

set(TARGET_NAME ${PROJECT_NAME})

file(GLOB_RECURSE SOURCE_LIST
	BSP/*.c BSP/*.h BSP/*.cpp
	Components/*.c Components/*.h Components/*.cpp
	Core/*.c Core/*.h Core/*.cpp Core/*.s
	damc/*.c damc/*.h damc/*.cpp
	Drivers/*.c Drivers/*.h Drivers/*.cpp
	Middlewares/*.c Middlewares/*.h Middlewares/*.cpp
	USB_DEVICE/*.c USB_DEVICE/*.h USB_DEVICE/*.cpp
)

add_executable(${TARGET_NAME} ${SOURCE_LIST})
add_st_target_properties(${TARGET_NAME})


target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:DEBUG>"
    "$<$<CONFIG:Debug>:USE_FULL_ASSERT>"
    USE_STM32F723E_DISCO_REVD
    USE_HAL_DRIVER
    STM32F723xx
)

target_include_directories(
    ${TARGET_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/Core/Inc
    ${PROJECT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
    ${PROJECT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${PROJECT_SOURCE_DIR}/USB_DEVICE/App
    ${PROJECT_SOURCE_DIR}/USB_DEVICE/Target
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc
    ${PROJECT_SOURCE_DIR}/BSP
    ${PROJECT_SOURCE_DIR}/damc/deps
    ${PROJECT_SOURCE_DIR}/damc/damc_audio_processing
    ${PROJECT_SOURCE_DIR}/damc/damc_common
    ${PROJECT_SOURCE_DIR}/damc/damc_simple_lib
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/COMPOSITE/Inc
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

target_compile_options(
    ${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:-g3>"
    "$<$<NOT:$<CONFIG:Debug>>:-g>"
    "$<$<NOT:$<CONFIG:Debug>>:-O3>"
	-Wall
	-Wno-psabi
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    -T ${PROJECT_SOURCE_DIR}/STM32F723IEKX_FLASH.ld
)
