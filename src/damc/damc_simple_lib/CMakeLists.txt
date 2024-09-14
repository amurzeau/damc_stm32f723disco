cmake_minimum_required(VERSION 3.1)

set(TARGET_NAME damc_simple_lib)

add_library(${TARGET_NAME}
	OscSerialClient.cpp
	OscSerialClient.h
	AudioProcessor.cpp
	AudioProcessor.h
)
target_link_libraries(${TARGET_NAME} PUBLIC damc_common damc_audio_processing)
target_compile_definitions(${TARGET_NAME} PRIVATE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS NOMINMAX JSON_SKIP_UNSUPPORTED_COMPILER_CHECK)

install(TARGETS
	${TARGET_NAME}
	RUNTIME DESTINATION ./
	LIBRARY DESTINATION lib/
)
