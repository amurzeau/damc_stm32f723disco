set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(WINDOWS_ST_CLT_PATH "C:/ST/STM32CubeCLT/STM32CubeCLT/GNU-tools-for-STM32/bin/")
set(MAC_ST_CLT_PATH "/opt/ST/STM32CubeCLT/GNU-tools-for-STM32/bin/")
set(LINUX_ST_CLT_PATH "/opt/st/stm32cubeclt/GNU-tools-for-STM32/bin")

# Try to find an STM32CubeIDE installation to use for the toolchain.
file(GLOB TOOLCHAIN_DIRECTORIES
	"C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/"
	"/opt/st/stm32cubeide*/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/"
	"/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/"
)

find_program(COMPILER_PATH
	NAMES arm-none-eabi-gcc
	PATHS
		${WINDOWS_ST_CLT_PATH}
		${MAC_ST_CLT_PATH}
		${LINUX_ST_CLT_PATH}
		${TOOLCHAIN_DIRECTORIES}
	REQUIRED
)
get_filename_component(TOOLCHAIN_DIRECTORY ${COMPILER_PATH} DIRECTORY)

if(WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
endif()

set(TOOLCHAIN_PREFIX                "${TOOLCHAIN_DIRECTORY}/arm-none-eabi-")

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX})
set(CMAKE_OBJDUMP                   ${TOOLCHAIN_PREFIX}objdump${TOOLCHAIN_SUFFIX})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX})

set(COMPILER_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb --specs=nano.specs -g3 -fdata-sections -ffunction-sections -fstack-usage -fcyclomatic-complexity")

set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static -Wl,--gc-sections")
set(CMAKE_ASM_FLAGS_INIT        "-x assembler-with-cpp")
set(CMAKE_C_FLAGS_INIT          "${COMPILER_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT        "${COMPILER_FLAGS} -fno-rtti -fno-exceptions")
set(CMAKE_C_FLAGS_DEBUG    "" CACHE STRING "Flags used by the C compiler during RELWITHDEBINFO builds.")
set(CMAKE_CXX_FLAGS_DEBUG  "" CACHE STRING "Flags used by the CXX compiler during DEBUG builds.")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -DNDEBUG" CACHE STRING "Flags used by the C compiler during RELWITHDEBINFO builds.")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -DNDEBUG" CACHE STRING "Flags used by the CXX compiler during RELWITHDEBINFO builds.")
set(CMAKE_C_FLAGS_RELEASE "-O3" CACHE STRING "Flags used by the C compiler during RELEASE builds.")
set(CMAKE_CXX_FLAGS_RELEASE "-O3" CACHE STRING "Flags used by the CXX compiler during RELEASE builds.")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)