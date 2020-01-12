cmake_minimum_required(VERSION 3.15)
project(ti_mac_coprocessor_host C)

set(CMAKE_C_STANDARD 11)

include(mcp_host.cmake)

add_executable(mcp_host_tests
        mcp_host_tests.c
        ${MCP_HOST_SOURCE_FILES})

target_include_directories(mcp_host_tests PRIVATE ${MCP_HOST_INCLUDE_DIRS})
# TODO: Add/fix warnings
#target_compile_options(mcp_host_tests PRIVATE -Wall -Wextra -pedantic -Werror)
target_compile_definitions(mcp_host_tests PRIVATE -DCOPROCESSOR -DOSAL_PORT2TIRTOS)
