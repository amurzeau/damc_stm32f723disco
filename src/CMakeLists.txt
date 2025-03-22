cmake_minimum_required(VERSION 3.20)

include(cmake/st-project.cmake)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${TOOLCHAIN_TARGET_NAME})

set(DISABLEFLOAT16 ON)
add_subdirectory(CMSIS-DSP)
target_link_libraries(CMSISDSP PUBLIC stm32cube_headers)


file(GLOB_RECURSE SOURCE_LIST
	USBClass/*.c USBClass/*.h USBClass/*.cpp
	damc/*.c damc/*.h damc/*.cpp
	memtester/*.c memtester/*.h memtester/*.cpp
)

add_executable(${TARGET_NAME} ${SOURCE_LIST})
add_st_target_properties(${TARGET_NAME})
target_link_libraries(${TARGET_NAME} PUBLIC CMSISDSP)
target_link_libraries(${TARGET_NAME} PUBLIC stm32cube)

target_include_directories(
    ${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/USBClass/AUDIO/Inc
    ${CMAKE_CURRENT_LIST_DIR}/USBClass/COMPOSITE/Inc
    ${CMAKE_CURRENT_LIST_DIR}/USBClass/CDC/Inc
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
