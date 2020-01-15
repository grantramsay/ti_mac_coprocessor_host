cmake_minimum_required(VERSION 3.15)
project(ti_mac_coprocessor_host C)

set(CMAKE_C_STANDARD 11)

include(mcp_host.cmake)

add_executable(mcp_host_tests
        mcp_host_tests.c
        osal_posix.c
        ${MCP_HOST_SOURCE_FILES})

target_include_directories(mcp_host_tests PRIVATE ${MCP_HOST_INCLUDE_DIRS})
target_compile_options(mcp_host_tests PRIVATE -Wall -Wextra -pedantic -Werror)
target_link_libraries(mcp_host_tests pthread)


add_executable(posix_coprocessor_host
        posix_coprocessor_host.c
        osal_posix.c
        ${MCP_HOST_SOURCE_FILES})

target_include_directories(posix_coprocessor_host PRIVATE ${MCP_HOST_INCLUDE_DIRS})
target_compile_options(posix_coprocessor_host PRIVATE -Wall -Wextra -pedantic -Werror)
target_link_libraries(posix_coprocessor_host pthread)
