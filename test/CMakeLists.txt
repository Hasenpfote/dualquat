cmake_minimum_required(VERSION 3.13)

include(DownloadProject/DownloadProject)

# eigen
download_project(
    PROJ                eigen
    GIT_REPOSITORY      https://gitlab.com/libeigen/eigen.git
    GIT_TAG             master
    UPDATE_DISCONNECTED 1
    )

# googletest
download_project(
    PROJ                googletest
    GIT_REPOSITORY      https://github.com/google/googletest.git
    GIT_TAG             master
    UPDATE_DISCONNECTED 1
    )

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(${FORCE_32BIT_BUILD})
    set(CMAKE_CXX_FLAGS -m32)
endif()

add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

###############################################################################
# Testing
###############################################################################
enable_testing()

set(TEST_NAME "${PROJECT_NAME}_tests")
set(TEST_LABELS "${PROJECT_NAME}")

add_executable(${TEST_NAME})

target_sources(
    ${TEST_NAME}
    PRIVATE
    gtest_helper.h
    gtest_helper.cpp
    test_relational.cpp
    test_quat_relational.cpp
    test_quat_exponential.cpp
    test_dualquat_base.cpp
    test_dualquat_relational.cpp
    test_dualquat_query.cpp
    test_dualquat_common.cpp
    test_dualquat_exponential.cpp
    test_dualquat_transformation.cpp
    test_dualquat_helper.cpp
    # Add a new file here.
    )

target_include_directories(
    ${TEST_NAME}
    SYSTEM PRIVATE
    ${eigen_SOURCE_DIR}
    PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
    )

target_compile_features(
    ${TEST_NAME}
    PRIVATE
    cxx_std_11
    )

set_target_properties(
    ${TEST_NAME}
    PROPERTIES
    CXX_EXTENSIONS OFF
    )

target_link_libraries(
    ${TEST_NAME}
    gtest
    gmock_main
    )
    
target_compile_definitions(
    ${TEST_NAME}
    PRIVATE
    -D_UNICODE
    -DUNICODE
    )

target_compile_options(
    ${TEST_NAME}
    PRIVATE
    # MSVC
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    # GNU
    #$<$<CXX_COMPILER_ID:GNU>:-Werror>
    $<$<CXX_COMPILER_ID:GNU>:-pedantic>
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CXX_COMPILER_ID:GNU>:-Wextra>
    $<$<CXX_COMPILER_ID:GNU>:-Wcast-align>
    $<$<CXX_COMPILER_ID:GNU>:-Wcast-qual>
    $<$<CXX_COMPILER_ID:GNU>:-Wctor-dtor-privacy>
    $<$<CXX_COMPILER_ID:GNU>:-Wdisabled-optimization>
    $<$<CXX_COMPILER_ID:GNU>:-Wformat=2>
    $<$<CXX_COMPILER_ID:GNU>:-Winit-self>
    $<$<CXX_COMPILER_ID:GNU>:-Wlogical-op>
    $<$<CXX_COMPILER_ID:GNU>:-Wmissing-declarations>
    $<$<CXX_COMPILER_ID:GNU>:-Wmissing-include-dirs>
    $<$<CXX_COMPILER_ID:GNU>:-Wnoexcept>
    $<$<CXX_COMPILER_ID:GNU>:-Wold-style-cast>
    $<$<CXX_COMPILER_ID:GNU>:-Woverloaded-virtual>
    $<$<CXX_COMPILER_ID:GNU>:-Wredundant-decls>
    $<$<CXX_COMPILER_ID:GNU>:-Wshadow>
    $<$<CXX_COMPILER_ID:GNU>:-Wsign-conversion>
    $<$<CXX_COMPILER_ID:GNU>:-Wsign-promo>
    $<$<CXX_COMPILER_ID:GNU>:-Wstrict-null-sentinel>
    $<$<CXX_COMPILER_ID:GNU>:-Wstrict-overflow=5>
    $<$<CXX_COMPILER_ID:GNU>:-Wswitch-default>
    $<$<CXX_COMPILER_ID:GNU>:-Wundef>
    #$<$<CXX_COMPILER_ID:GNU>:-Wno-unused>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:DEBUG>>:-O0 -g3 -pg>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:RELEASE>>:-O2 -s -DNDEBUG -march=native>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:MINSIZEREL>>:-Os -s -DNDEBUG -march=native>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:RELWITHDEBINFO>>:-Og -g3 -pg>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<BOOL:${FORCE_32BIT_BUILD}>>:-m32>
    # Clang
    #$<$<CXX_COMPILER_ID:Clang>:-Werror>
    $<$<CXX_COMPILER_ID:Clang>:-Weverything>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-c++98-compat>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-c++98-compat-pedantic>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-covered-switch-default>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-disabled-macro-expansion>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-global-constructors>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-inconsistent-missing-destructor-override>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-padded>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-macros>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-member-function>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-weak-vtables>
    $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:DEBUG>>:-O0 -g3 -pg>
    $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:RELEASE>>:-O2 -DNDEBUG -march=native>
    $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:MINSIZEREL>>:-Os -DNDEBUG -march=native>
    $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:RELWITHDEBINFO>>:-Og -g3 -pg>
    $<$<AND:$<CXX_COMPILER_ID:Clang>,$<BOOL:${FORCE_32BIT_BUILD}>>:-m32>
    )

target_link_options(
    ${TEST_NAME}
    PRIVATE
    # GNU
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<BOOL:${FORCE_32BIT_BUILD}>>:-m32>
    # Clang
    $<$<AND:$<CXX_COMPILER_ID:Clang>,$<BOOL:${FORCE_32BIT_BUILD}>>:-m32>
    )

add_test(
    NAME ${TEST_NAME}
    COMMAND $<TARGET_FILE:${TEST_NAME}>
    )

# run with: ctest -L xxx
set_tests_properties(
    ${TEST_NAME}
    PROPERTIES
    LABELS ${TEST_LABELS}
    )
