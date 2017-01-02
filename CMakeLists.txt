cmake_minimum_required(VERSION 3.5)
project(8086tiny)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES
        8086tiny.cpp
        )

link_libraries(SDL)
add_executable(8086tiny ${SOURCE_FILES})

include(PVS-Studio.cmake)
pvs_studio_add_target(TARGET analyze ALL
        FORMAT tasklist
        #PREPROCESSOR gcc
        LOG "PVS-report.tasks"
        ANALYZE 8086tiny
        #LICENSE "PVS-Studio.lic"
        #CXX_FLAGS ${PREPROCESSOR_ADDITIONAL_FLAGS}
        #C_FLAGS ${PREPROCESSOR_ADDITIONAL_FLAGS}
        #CONFIG "PVS-Studio.cfg"
        )
