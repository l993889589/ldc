cmake_minimum_required(VERSION 3.16)

project(ldc VERSION 2.0.2 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(ldc STATIC ldc.c)

target_include_directories(ldc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(MSVC)
    target_compile_options(ldc PRIVATE /W4 /WX)
else()
    target_compile_options(ldc PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

option(LDC_BUILD_TESTS "Build LDC host tests" ON)

if(LDC_BUILD_TESTS)
    enable_testing()
    add_executable(test_ldc tests/test_ldc.c)
    target_link_libraries(test_ldc PRIVATE ldc)
    if(MSVC)
        target_compile_options(test_ldc PRIVATE /W4 /WX)
    else()
        target_compile_options(test_ldc PRIVATE
            -Wall -Wextra -Wpedantic -Werror
        )
    endif()
    add_test(NAME ldc_all COMMAND test_ldc)
endif()
