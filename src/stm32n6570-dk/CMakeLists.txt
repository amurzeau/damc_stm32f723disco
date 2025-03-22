cmake_minimum_required(VERSION 3.20)


add_library(stm32cube_headers INTERFACE)

target_include_directories(stm32cube_headers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/FSBL/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32N6xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32N6xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
)

target_compile_options(stm32cube_headers INTERFACE
	-ggdb3
	-gdwarf-4
	"$<$<CONFIG:Release>:-Ofast>"
	"$<$<CONFIG:Release>:-flto>"
)

target_compile_definitions(stm32cube_headers INTERFACE
    "$<$<CONFIG:Debug>:DEBUG>"
    "$<$<CONFIG:Debug>:USE_FULL_ASSERT>"
    USE_HAL_DRIVER
    STM32N657xx
)


file(GLOB_RECURSE SOURCE_LIST
	FSBL/BSP/*.c
	FSBL/BSP/*.h
	FSBL/BSP/*.cpp
	FSBL/Components/*.c
	FSBL/Components/*.h
	FSBL/Components/*.cpp
	FSBL/Core/*.c
	FSBL/Core/*.h
	FSBL/Core/*.cpp
	FSBL/Core/*.s
	Drivers/*.c
	Drivers/*.h
	Drivers/*.cpp
	FSBL/USB_DEVICE/*.c
	FSBL/USB_DEVICE/*.h
	FSBL/USB_DEVICE/*.cpp
	FSBL/Middlewares/ST/STM32_USB_Device_Library/Core/*.c
	FSBL/Middlewares/ST/STM32_USB_Device_Library/Core/*.h
	FSBL/Middlewares/ST/STM32_USB_Device_Library/Core/*.cpp
)

add_library(stm32cube INTERFACE)
target_sources(stm32cube INTERFACE ${SOURCE_LIST})

target_include_directories(
    stm32cube INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/FSBL/USB_DEVICE/App
    ${CMAKE_CURRENT_LIST_DIR}/FSBL/USB_DEVICE/Target
    ${CMAKE_CURRENT_LIST_DIR}/FSBL/BSP
    ${CMAKE_CURRENT_LIST_DIR}/FSBL/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
)

target_link_libraries(stm32cube INTERFACE stm32cube_headers)

target_link_options(stm32cube INTERFACE
    -T ${CMAKE_CURRENT_LIST_DIR}/FSBL/STM32N657X0HXQ_AXISRAM2_fsbl.ld
)
set_target_properties(stm32cube PROPERTIES INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/FSBL/STM32N657X0HXQ_AXISRAM2_fsbl.ld)
