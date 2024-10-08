#==============================
# Requirements
#==============================

cmake_minimum_required(VERSION 3.10)

#==============================
# Project Settings
#==============================

set(TARGET_NAME "EasySerialPorting")
set(TARGET_NAMESPACE "")
set(TARGET_VERSION 1.0.0)

if (NOT PROJECT_DECLARED)
    project("${TARGET_NAMESPACE} ${TARGET_NAME}" LANGUAGES CXX VERSION ${TARGET_VERSION})
    set(PROJECT_DECLARED)
endif()

#==============================
# Command Lines
#==============================

set(CMAKE_CXX_STANDARD 17)

#------------------------------
# C++
#------------------------------

# C++ Source Files
FILE(GLOB_RECURSE TARGET_SOURCE *.cpp)
# C++ Header Files
FILE(GLOB_RECURSE TARGET_HEADER *.hpp)

#==============================
# Compile Targets
#==============================

add_library(${TARGET_NAME} STATIC ${TARGET_SOURCE} ${TARGET_HEADER})

# Set version property.
set_target_properties(${TARGET_NAME} PROPERTIES
        VERSION ${TARGET_VERSION}
        OUTPUT_NAME ${TARGET_NAMESPACE}${TARGET_NAME}
        DEBUG_POSTFIX "d")

# Enable 'DEBUG' Macro in Debug Mode
if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_compile_definitions(${TARGET_NAME} PRIVATE -DDEBUG)
endif()

#==============================
# Dependencies
#==============================

# Boost
find_package(Boost 1.65 REQUIRED)
target_include_directories(${TARGET_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} PUBLIC ${Boost_LIBRARIES})

# In Linux, 'Threads' need to explicitly linked.
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(${TARGET_NAME} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
    target_link_libraries(${TARGET_NAME} PUBLIC dl)
endif()

#===============================
# Install Scripts
#===============================

include(CMakePackageConfigHelpers)

configure_file(PackageVersion.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAMESPACE}${TARGET_NAME}Version.hpp)

set(CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/${TARGET_NAMESPACE}${TARGET_NAME})

configure_package_config_file(
        PackageConfig.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAMESPACE}${TARGET_NAME}Config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${TARGET_NAMESPACE}${TARGET_NAME}
        PATH_VARS TARGET_NAMESPACE TARGET_NAME CONFIG_INSTALL_PATH)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAMESPACE}${TARGET_NAME}ConfigVersion.cmake
        VERSION ${TARGET_VERSION}
        COMPATIBILITY SameMajorVersion)

# Install executable files and libraries to 'default_path/'.
install(TARGETS ${TARGET_NAME}
        EXPORT ${TARGET_NAMESPACE}${TARGET_NAME}Targets
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY "." DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${TARGET_NAMESPACE}${TARGET_NAME}/ FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY "." DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${TARGET_NAMESPACE}${TARGET_NAME}/ FILES_MATCHING PATTERN "*.h")
install(DIRECTORY "." DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${TARGET_NAMESPACE}${TARGET_NAME}/ FILES_MATCHING PATTERN "*.cuh")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAMESPACE}${TARGET_NAME}Version.hpp
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${TARGET_NAMESPACE}${TARGET_NAME}/)
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAMESPACE}${TARGET_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAMESPACE}${TARGET_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${TARGET_NAMESPACE}${TARGET_NAME})
install(EXPORT ${TARGET_NAMESPACE}${TARGET_NAME}Targets
        NAMESPACE ${TARGET_NAMESPACE}::
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${TARGET_NAMESPACE}${TARGET_NAME})