cmake_minimum_required(VERSION 3.13)

# Extend CMake module path for loading custom modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(UNIX)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Force a Release build." FORCE)
    endif()
    option(FORCE_32BIT_BUILD "Force a 32bit compile on 64bit" OFF)
endif()

###############################################################################
# Project settings
###############################################################################
project(
    "dualquat"
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "A header only c++ library."
    )

add_library(${PROJECT_NAME} INTERFACE)
# add alias so the project can be uses with add_subdirectory.
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

include(GNUInstallDirs)
include(CTest)

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

target_compile_features(
    ${PROJECT_NAME}
    INTERFACE
    cxx_std_11
    )

###############################################################################
# Testing
###############################################################################
option(DUALQUAD_BUILD_TESTING "Build and run tests." ON)

if(${DUALQUAD_BUILD_TESTING})
    add_subdirectory(test)
endif()

###############################################################################
# Installation settings
###############################################################################
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_Targets
    #ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    )

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
    )

option(DUALQUAD_INSTALL_LIBRARY "Enable installing of library into default locations." OFF)

if(${DUALQUAD_INSTALL_LIBRARY})
    install(
        EXPORT ${PROJECT_NAME}_Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
        )

    install(
        FILES
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
        )

    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/include/dualquat
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
endif()
