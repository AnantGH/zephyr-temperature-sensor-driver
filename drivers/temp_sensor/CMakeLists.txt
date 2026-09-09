zephyr_library()

# List the source code files for the library
zephyr_library_sources(temp_sen.c)

# Add header files to the CMake search directories
zephyr_include_directories(.)