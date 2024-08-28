function(add_st_target_properties TARGET_NAME)
	target_link_options(${TARGET_NAME} PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.map)
	add_custom_command(
		TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
	)

	add_custom_command(
		TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O ihex
		$<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
	)

	add_custom_command(
		TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O binary
		$<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
	)

	add_custom_command(
		TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:${TARGET_NAME}> > ${TARGET_NAME}.list
	)
endfunction()
