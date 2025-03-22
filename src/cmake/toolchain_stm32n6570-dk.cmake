cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_LIST_DIR}/toolchain_common.cmake)

define_toolchain(stm32n6570-dk "-mcpu=cortex-m55 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb --specs=nano.specs -g3 -fdata-sections -ffunction-sections -fstack-usage -fstack-protector -mcmse")
