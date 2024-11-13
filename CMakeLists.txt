cmake_minimum_required(VERSION 3.10)

# Project Name and Language
project(radar_project LANGUAGES CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# Include Directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/quill/include
)

# Add Subdirectory for Quill Library
add_subdirectory(lib/quill)

# Add Executable Target
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/radar.cpp
    #    src/angular_speed_estimator.cpp
    src/logger/logger.cpp
    # Add other source files if needed
)

# Link Libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    quill::quill
    pthread
)

# Set Include Directories for Target
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Compiler Options
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -Werror=return-type)
endif()

# Set Output Directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

