# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(
	wifi_gspi_merged
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_wifi_gspi_merged target,
# which contains the content of the SLC project
include(wifi_gspi_merged.cmake)

add_executable(wifi_gspi_merged
    # Add additional sources here
    ../src/lr1121_hw_test.c
)

target_include_directories(wifi_gspi_merged PUBLIC
    # Add additional include paths here
)

target_compile_definitions(wifi_gspi_merged PUBLIC
    # Add additional macros here
)

target_compile_options(wifi_gspi_merged PUBLIC
    # Set additional compiler flags here
)

target_link_options(wifi_gspi_merged PUBLIC
    # Set additional linker flags here
)

# Link with the content defined in the SLC project
target_link_libraries(wifi_gspi_merged PRIVATE
    slc
)

# Include managed project content if available
include(wifi_gspi_merged_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(wifi_gspi_merged PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET wifi_gspi_merged
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET wifi_gspi_merged
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()
