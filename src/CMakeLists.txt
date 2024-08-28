cmake_minimum_required(VERSION 3.20)

include(cmake/st-project.cmake)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(
	-ggdb3
	-gdwarf-4
	"$<$<CONFIG:Release>:-Ofast>"
	"$<$<CONFIG:Release>:-flto>"
)

add_compile_definitions(
    "$<$<CONFIG:Debug>:DEBUG>"
    "$<$<CONFIG:Debug>:USE_FULL_ASSERT>"
    USE_STM32F723E_DISCO_REVD
    USE_HAL_DRIVER
    STM32F723xx
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
)

set(DISABLEFLOAT16 ON)
add_subdirectory(CMSIS-DSP)

file(GLOB_RECURSE SOURCE_LIST
	BSP/*.c BSP/*.h BSP/*.cpp
	Components/*.c Components/*.h Components/*.cpp
	Core/*.c Core/*.h Core/*.cpp Core/*.s
	damc/*.c damc/*.h damc/*.cpp
	Drivers/*.c Drivers/*.h Drivers/*.cpp
	Middlewares/*.c Middlewares/*.h Middlewares/*.cpp
	USB_DEVICE/*.c USB_DEVICE/*.h USB_DEVICE/*.cpp
	memtester/*.c memtester/*.h memtester/*.cpp
)

add_executable(${TARGET_NAME} ${SOURCE_LIST})
add_st_target_properties(${TARGET_NAME})
target_link_libraries(${TARGET_NAME} PUBLIC CMSISDSP)


target_include_directories(
    ${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/USB_DEVICE/App
    ${CMAKE_CURRENT_LIST_DIR}/USB_DEVICE/Target
    ${CMAKE_CURRENT_LIST_DIR}/BSP
    ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/COMPOSITE/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_CURRENT_LIST_DIR}/damc/deps
    ${CMAKE_CURRENT_LIST_DIR}/damc/deps/libuv_microcontroller
    ${CMAKE_CURRENT_LIST_DIR}/damc/damc_audio_processing
    ${CMAKE_CURRENT_LIST_DIR}/damc/damc_common
    ${CMAKE_CURRENT_LIST_DIR}/damc/damc_simple_lib
    ${CMAKE_CURRENT_LIST_DIR}/memtester
)

target_compile_options(
    ${TARGET_NAME} PRIVATE
	-Wall
	-Wno-psabi
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    -T ${CMAKE_CURRENT_LIST_DIR}/STM32F723IEKX_FLASH.ld
)
set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/STM32F723IEKX_FLASH.ld)
