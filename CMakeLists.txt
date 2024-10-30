cmake_minimum_required(VERSION 3.10)

# Register general cmake commands
include(cmake/Custom.cmake)

# Set policies
cmake_policy(SET CMP0054 NEW) # Only interpret if() arguments as variables or keywords when unquoted.
cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH is enabled by default.
cmake_policy(SET CMP0063 NEW) # Honor visibility properties for all target types.
cmake_policy(SET CMP0077 NEW) # option() honors normal variables.
cmake_policy(SET CMP0120 OLD) # The WriteCompilerDetectionHeader module is removed.

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)  # You can choose 11, 14, or 17 depending on your needs
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add message color headers

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldBlue    "${Esc}[1;90m")
  set(Reset "${Esc}[0m")  # ANSI code for bright and bold green

endif()



# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(GenerateExportHeader)

set(WriterCompilerDetectionHeaderFound NOTFOUND)

# This module is only available with CMake >=3.1, so check whether it could be found
# BUT in CMake 3.1 this module doesn't recognize AppleClang as compiler, so just use it as of CMake 3.2
if (${CMAKE_VERSION} VERSION_GREATER "3.2")
    include(WriteCompilerDetectionHeader OPTIONAL RESULT_VARIABLE WriterCompilerDetectionHeaderFound)
endif()

# Include custom cmake modules
include(cmake/Coverage.cmake)
include(cmake/GenerateTemplateExportHeader.cmake)
include(cmake/GetGitRevisionDescription.cmake)
include(cmake/HealthCheck.cmake)

# 
# Project description and (meta) information
# 

# Get git revision
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
string(SUBSTRING "${GIT_SHA1}" 0 12 GIT_REV)
if(NOT GIT_SHA1)
    set(GIT_REV "0")
endif()

# Check if the platform is Windows or Unix-based and set the BUILD_DATE accordingly
if(WIN32)
  # Windows command to get time with milliseconds
  execute_process(
      COMMAND powershell -Command "[DateTime]::Now.ToString('yyyy-MM-dd HH:mm:ss.fff')"
      OUTPUT_VARIABLE BUILD_DATE
  )
else()
  # Unix-based command to get time with milliseconds
  execute_process(
      COMMAND bash -c "echo -n $(date '+%Y-%m-%d %H:%M:%S.%3N')"
      OUTPUT_VARIABLE BUILD_DATE
  )
endif()

# Add the BUILD_DATE definition
add_definitions(-DBUILD_DATE="${BUILD_DATE}")

# Meta information about the project
set(META_PROJECT_NAME        "radar")
set(META_PROJECT_DESCRIPTION "Halo Radar")
set(META_AUTHOR_ORGANIZATION "University of Notre Dame")
set(META_AUTHOR_DOMAIN       "")
set(META_AUTHOR_MAINTAINER   "")
set(META_VERSION_MAJOR       "1")
set(META_VERSION_MINOR       "0")
set(META_VERSION_PATCH       "0")
set(META_VERSION_REVISION    "${GIT_REV}")
set(META_VERSION             "${META_VERSION_MAJOR}.${META_VERSION_MINOR}.${META_VERSION_PATCH}")
set(META_NAME_VERSION        "${META_PROJECT_NAME} v${META_VERSION} (${META_VERSION_REVISION})")
set(META_CMAKE_INIT_SHA      "${GIT_REV}")

string(MAKE_C_IDENTIFIER ${META_PROJECT_NAME} META_PROJECT_ID)
string(TOUPPER ${META_PROJECT_ID} META_PROJECT_ID)

set_target_properties(${PROJECT_NAME} PROPERTIES
                       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Configure the version file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.h
)

#
# Project configuration options
# 

# Project options
# option(BUILD_SHARED_LIBS      "Build shared instead of static libraries."              ON)
# option(OPTION_SELF_CONTAINED  "Create a self-contained install with all dependencies." OFF)
# option(OPTION_BUILD_TESTS     "Build tests."                                           ON)
# option(OPTION_BUILD_DOCS      "Build documentation."                                   OFF)
# option(OPTION_BUILD_EXAMPLES  "Build examples."                                        OFF)
option(OPTION_ENABLE_COVERAGE "Add coverage information."                              ON)
option(SERVER_CONNECT "Connect Without GUI" OFF)


#
# Project Health Check Setup
#

# Add cmake-init template check cmake targets
add_check_template_target(${META_CMAKE_INIT_SHA})

# Configure health check tools
enable_cppcheck(ON)
enable_clang_tidy(ON)
enable_coverage(${OPTION_ENABLE_COVERAGE})

# Project-wide include directories. Add any new general include directories here.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${OPENSSL_INCLUDE_DIR})

# Quill 

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/quill)

# Project declaration must be after the Hunter Cmake build system.
project(RADAR LANGUAGES CXX)

# Define the executable target. 
# Add new source files to this list to compile them as part of the project.
add_executable(${PROJECT_NAME} 
    src/radar.cpp
    src/radar_node.cpp
    src/logger/logger.cpp
)

# Set include directories for the target. 
# Add specific include directories related to this target.
target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries with the target. 
# Add new libraries to link against by appending them to this list.
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    quill::quill
)
# Specify C++17 as the target C++ standard for this project.
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# Compiler options for non-MSVC compilers. 
# Add or modify options as necessary.
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -Werror=return-type)
endif()

# Set default build type to Release if not specified. 
# Change this to Debug or another type if needed.
set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E env bash ${CMAKE_SOURCE_DIR}/scripts/post_build_script.sh
)



# Windows-specific configuration to handle runtime DLLs, crucial for deployment.
if(WIN32)
    # Ensure the correct handling of runtime DLLs.
    if(CMAKE_VERSION VERSION_LESS "3.21")
        message(WARNING "CMake version less than 3.21 detected. Automatic DLL handling is not available. Manually ensure DLLs are copied to the executable directory.")
    else()
        # Automatically copy required DLLs next to the executable.
        #  Useful for deployment and distribution.
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD 
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> 
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
            COMMAND_EXPAND_LISTS
        )
    endif()
endif()

# Documentation Generation
# Find Doxygen package
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Configure the Doxyfile
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # Add a custom target to generate documentation
    add_custom_target(doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif()