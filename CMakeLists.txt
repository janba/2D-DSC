cmake_minimum_required(VERSION 3.6)
project(2D-DSC)

set(CMAKE_CXX_STANDARD 14)

if (APPLE)
    # TODO for Mac
endif (APPLE)

if (UNIX)
    # TODO for UNix
endif (UNIX)
if(WIN32)
    # TODO for Windows
endif()


aux_source_directory(./src SRC)

include_directories(../GEL/src/GEL)

link_directories(../GEL)

add_library(2D-DSC SHARED ${SRC})
target_link_libraries(2D-DSC GEL)