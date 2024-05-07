cmake_minimum_required(VERSION 3.20)

find_package(Git QUIET)
if(GIT_FOUND)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		OUTPUT_VARIABLE DAMC_GIT_VERSION
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	string(REGEX REPLACE "^v?([0-9]+\\.[0-9]+(\\.[0-9]+)?).*\$" "\\1" DAMC_GIT_VERSION_TRIMMED ${DAMC_GIT_VERSION})
else()
	set(DAMC_GIT_VERSION_TRIMMED "0.0.0")
	set(DAMC_GIT_VERSION "unknown-rev")
endif()

project(damc_stm32f723disco VERSION "${DAMC_GIT_VERSION_TRIMMED}" LANGUAGES C CXX ASM)

include(cmake/st-project.cmake)

set(CMAKE_CXX_STANDARD 17)

set(TARGET_NAME ${PROJECT_NAME})

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
    ${PROJECT_SOURCE_DIR}/Core/Inc
    ${PROJECT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
    ${PROJECT_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Include
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
    ${PROJECT_SOURCE_DIR}/USB_DEVICE/App
    ${PROJECT_SOURCE_DIR}/USB_DEVICE/Target
    ${PROJECT_SOURCE_DIR}/BSP
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/COMPOSITE/Inc
    ${PROJECT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${PROJECT_SOURCE_DIR}/damc/deps
    ${PROJECT_SOURCE_DIR}/damc/damc_audio_processing
    ${PROJECT_SOURCE_DIR}/damc/damc_common
    ${PROJECT_SOURCE_DIR}/damc/damc_simple_lib
    ${PROJECT_SOURCE_DIR}/memtester
)

target_compile_options(
    ${TARGET_NAME} PRIVATE
	-Wall
	-Wno-psabi
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    -T ${PROJECT_SOURCE_DIR}/STM32F723IEKX_FLASH.ld
)
set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS ${PROJECT_SOURCE_DIR}/STM32F723IEKX_FLASH.ld)

install(FILES
	$<TARGET_FILE:${TARGET_NAME}>
	$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.bin
	$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.hex
	$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.list
	DESTINATION ./
	COMPONENT all
)

install(FILES
	$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.bin
	$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.hex
	dist/README.stlink_dragdrop.txt
	DESTINATION ./
	COMPONENT stlink_dragdrop
)

install(FILES
	$<TARGET_FILE:${TARGET_NAME}>
	dist/README.stm32cubeprogrammer.txt
	DESTINATION ./
	COMPONENT stm32cubeprogrammer
)

# Packaging
set(CPACK_GENERATOR ZIP)
set(CPACK_STRIP_FILES ON)
set(CPACK_PACKAGE_VERSION ${DAMC_GIT_VERSION})
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

# Remove system name from filename (it is "Generic" and this is not useful to have it)
set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${CPACK_PACKAGE_VERSION}")

include(CPack)
